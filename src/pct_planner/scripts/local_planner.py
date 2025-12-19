#!/usr/bin/env python3
import os
import runpy
import sys


def main() -> None:
    ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    target = os.path.join(ws_root, "PCT_planner", "planner", "scripts", "local_planner.py")
    target_dir = os.path.dirname(target)
    os.chdir(target_dir)
    sys.path.insert(0, target_dir)
    sys.argv = [target] + sys.argv[1:]
    runpy.run_path(target, run_name="__main__")


if __name__ == "__main__":
    main()
