"""
Q-learning agent for DuckieRace charging decisions.

State  : (battery_bin, zone)  →  10 × 4 = 40 states
Actions: DRIVE=0, SEEK_CHARGE=1
Table  : saved/loaded as JSON so learning persists across runs.

Warm-start mirrors the conservative strategy (charge when battery < 55%).
"""

import json
import os
import random

import numpy as np

# ── zone indices ────────────────────────────────────────────────────────────
ZONE_TRACK = 0   # on the main oval
ZONE_GATE  = 1   # at the charge-gate entry (usb_cam_2 detection)
ZONE_FUEL  = 2   # inside the fuel zone
ZONE_MERGE = 3   # at the merge zone (leaving charger)

ZONE_NAMES = {ZONE_TRACK: "track", ZONE_GATE: "gate",
              ZONE_FUEL: "fuel",   ZONE_MERGE: "merge"}

# ── action indices ──────────────────────────────────────────────────────────
ACTION_DRIVE  = 0   # drive normally, no charging intent
ACTION_CHARGE = 1   # initiate / continue charging sequence

N_ACTIONS  = 2
N_BAT_BINS = 10   # 0=0-10%, 1=10-20%, …, 9=90-100%
N_ZONES    = 4

DEFAULT_TABLE_PATH = os.path.expanduser("~/.ros/duckierace_q_table.json")


class QAgent:
    """
    Tabular Q-learning agent.

    Learning parameters:
      lr      – learning rate (α)
      gamma   – discount factor (γ)
      epsilon – exploration probability (ε-greedy)
    """

    def __init__(self, lr=0.05, gamma=0.90, epsilon=0.10,
                 table_path=DEFAULT_TABLE_PATH):
        self.lr      = float(lr)
        self.gamma   = float(gamma)
        self.epsilon = float(epsilon)
        self.path    = table_path

        # Q[bat_bin, zone, action]
        self.q = np.zeros((N_BAT_BINS, N_ZONES, N_ACTIONS), dtype=np.float32)
        self._warm_start()
        self.load()   # overwrite with saved values if a table exists

        self._prev_state  = None
        self._prev_action = None

    # ── initialisation ───────────────────────────────────────────────────────

    def _warm_start(self):
        """
        Pre-fill Q-table from the conservative strategy so the robot behaves
        reasonably on the first run without any training episodes.
        Conservative rule: charge when battery_bin < 6 (i.e. battery < 60%).
        """
        for b in range(N_BAT_BINS):
            for z in range(N_ZONES):
                if b < 6:                       # low battery → prefer charging
                    self.q[b, z, ACTION_CHARGE] = 1.0
                    self.q[b, z, ACTION_DRIVE]  = 0.0
                else:                           # high battery → prefer driving
                    self.q[b, z, ACTION_DRIVE]  = 1.0
                    self.q[b, z, ACTION_CHARGE] = 0.0
        # Inside fuel zone: always prefer to charge (strong prior)
        self.q[:, ZONE_FUEL, ACTION_CHARGE] = 2.0
        self.q[:, ZONE_FUEL, ACTION_DRIVE]  = -1.0

    # ── state encoding ───────────────────────────────────────────────────────

    @staticmethod
    def encode_state(battery: float, zone: int):
        bat_bin = int(min(max(battery, 0.0), 99.9) / 10)
        return bat_bin, zone

    # ── decision ─────────────────────────────────────────────────────────────

    def select_action(self, battery: float, zone: int) -> int:
        """ε-greedy action selection. Records (state, action) for the update."""
        state = self.encode_state(battery, zone)
        if random.random() < self.epsilon:
            action = random.randint(0, N_ACTIONS - 1)
        else:
            action = int(np.argmax(self.q[state]))
        self._prev_state  = state
        self._prev_action = action
        return action

    # ── learning ─────────────────────────────────────────────────────────────

    def update(self, reward: float, next_battery: float, next_zone: int):
        """
        Call after every control cycle with the observed reward and new state.
        Uses the standard Q-learning (off-policy) update rule.
        """
        if self._prev_state is None:
            return
        s  = self._prev_state
        a  = self._prev_action
        ns = self.encode_state(next_battery, next_zone)

        td_target = reward + self.gamma * float(np.max(self.q[ns]))
        td_error  = td_target - float(self.q[s][a])
        self.q[s][a] += self.lr * td_error

    # ── persistence ──────────────────────────────────────────────────────────

    def save(self):
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        payload = {
            "q":       self.q.tolist(),
            "lr":      self.lr,
            "gamma":   self.gamma,
            "epsilon": self.epsilon,
        }
        with open(self.path, "w") as f:
            json.dump(payload, f, indent=2)

    def load(self):
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path) as f:
                data = json.load(f)
            loaded = np.array(data["q"], dtype=np.float32)
            if loaded.shape == self.q.shape:
                self.q = loaded
        except Exception:
            pass   # corrupt file → keep warm-start values

    # ── diagnostics ──────────────────────────────────────────────────────────

    def policy_summary(self) -> str:
        """Human-readable table: for each (bat%, zone) show preferred action."""
        rows = ["bat%   track      gate       fuel       merge"]
        for b in range(N_BAT_BINS):
            cells = []
            for z in range(N_ZONES):
                a = int(np.argmax(self.q[b, z]))
                cells.append("CHARGE" if a == ACTION_CHARGE else "drive ")
            rows.append("%2d0%%  %s  %s  %s  %s" % (b, *cells))
        return "\n".join(rows)
