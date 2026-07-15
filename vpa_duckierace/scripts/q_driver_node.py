#!/usr/bin/env python3
import os, sys
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)
from q_table.driver_node import main
if __name__ == "__main__":
    main()
