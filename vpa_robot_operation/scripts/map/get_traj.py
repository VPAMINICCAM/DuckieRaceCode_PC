map_id = {
    # (316, 318): [(1,0,0)], # straight accrossing, but this can always be guided, so release early
    (320, 318): [(0.35,-0.35,-1.57)],
    (316, 300): [(0.5, 0, 0),(0.9,0.75,1.5)],
    (300, 323): [(1.9,0,0)],
    (323, 321): [(0.4,-0.4,-1.55)],
    (321, 314): [(1.7,0,0)],
    (314, 313): [(1.15,0,0)],
    (313, 325): [(0.5,0.10,-0.2),(1.85,0.2,0)],
    (325, 305): [(0.4,-0.35,-1.57)],
    (305, 317): [(0.85,0,0),(1.25,0.4,1.55),(1.25,1,1.55)],
    (317, 327): [(0.3,0,0),(0.63, 0.35,1.5)],
    (327, 312): [(0.5,0,0)],
    (312, 303): [(0.75,0,0),(1.2,0.45,1.55),(1.2,0.85,1.55)],
    (303, 319): [(1.35,-0.35,-0.2)],
    (318, 327): [(1.8,0,0)],
    (318, 307): [(1,0,0),(1.8,0.6,1.51)]
}

phase_group = {
    0: [320], # entry, this is the virtual signal for entry, so the entering car does not hit the car on main lane
    1: [302,303,317,319,308,307,310,311,323,315,313,313,321,325,314], # this phase main inter go straight from e <--> w and its down stream also green.
    2: [304,309,323,315,312,318], # left turn for main inter e<-->w, and its down stream also green
    3: [300,316],
    4: [327,305],
    100: [330] # exit, always green
}

def get_trajectory(start_id, end_id):
    return map_id.get((start_id, end_id), [])

def possible_goals(start_id):
    goals = []
    for (s, e) in map_id.keys():
        if s == start_id:
            goals.append(e)
    return goals

def get_phase_group(start_id):
    # so far the lane should only be discharged in one phase, even they have multiple possible goals
    for pg, tags in phase_group.items():
        if start_id in tags:
            return pg
    return -1