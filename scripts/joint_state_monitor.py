#!/usr/bin/env python3
"""
UR5 Joint State Monitor
=======================
Subscribes to /model/ur5_robot/joint_states and prints a live table
of joint positions.  Useful for debugging and verifying motion.

Usage:
  conda activate gz_run
  python joint_state_monitor.py
  python joint_state_monitor.py --once   # print once and exit
"""

import sys
import time
import subprocess
import argparse
import json
import re
from typing import Dict, Optional

JOINT_STATE_TOPIC = "/model/ur5_robot/joint_states"

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_pan_joint",
    "elbow_lift_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "left_finger_joint",
    "right_finger_joint",
]


def echo_topic_once(topic: str, timeout: float = 5.0) -> Optional[str]:
    """Run 'gz topic -e -t <topic> --once' and return raw output."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-t", topic, "--once"],
            capture_output=True, text=True, timeout=timeout
        )
        return result.stdout if result.returncode == 0 else None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def parse_joint_state(raw: str) -> Dict[str, float]:
    """Parse protobuf text output from gz topic echo for joint states."""
    positions = {}
    # Pattern: name: "joint_name"  position: value
    name_pattern = re.compile(r'name:\s+"([^"]+)"')
    pos_pattern  = re.compile(r'position:\s+([\-0-9.e+]+)')

    names = name_pattern.findall(raw)
    pos_vals = pos_pattern.findall(raw)

    for name, pos in zip(names, pos_vals):
        try:
            positions[name] = float(pos)
        except ValueError:
            pass
    return positions


def print_joint_table(positions: Dict[str, float], show_deg: bool = True):
    """Print formatted joint state table."""
    print("\n" + "─" * 55)
    print(f"  {'Joint':<28}  {'Rad':>9}  {'Deg':>9}")
    print("─" * 55)
    for jname in JOINT_NAMES:
        val = positions.get(jname, float('nan'))
        deg = val * 180.0 / 3.14159 if not (val != val) else float('nan')
        is_gripper = "finger" in jname
        unit_str = f"{val*1000:.2f} mm" if is_gripper else f"{deg:+.2f}°"
        print(f"  {jname:<28}  {val:>+9.4f}  {unit_str:>9}")
    print("─" * 55)


def monitor_continuous(interval: float = 1.0):
    """Continuously poll and display joint states."""
    print("UR5 Joint State Monitor  (Ctrl+C to stop)")
    print(f"Polling topic: {JOINT_STATE_TOPIC}")
    print(f"Interval: {interval}s\n")

    while True:
        try:
            raw = echo_topic_once(JOINT_STATE_TOPIC, timeout=interval + 1.0)
            ts = time.strftime("%H:%M:%S")
            if raw:
                positions = parse_joint_state(raw)
                print(f"\n[{ts}] Joint States:")
                print_joint_table(positions)
            else:
                print(f"[{ts}] No data (is Gazebo running?)")
            time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")
            break


def monitor_once():
    """Print joint states once and exit."""
    print(f"Reading: {JOINT_STATE_TOPIC}")
    raw = echo_topic_once(JOINT_STATE_TOPIC)
    if raw:
        positions = parse_joint_state(raw)
        print_joint_table(positions)
    else:
        print("No data received. Is Gazebo running with UR5 model?")
        sys.exit(1)


def list_topics():
    """Print all active gz topics."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5
        )
        print("Active topics:")
        for line in result.stdout.splitlines():
            if "ur5" in line.lower():
                print(f"  ★ {line}")
            else:
                print(f"    {line}")
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"Error listing topics: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UR5 Joint State Monitor")
    parser.add_argument("--once",      action="store_true",
                        help="Read once and exit")
    parser.add_argument("--interval",  type=float, default=1.0,
                        help="Polling interval in seconds (default: 1.0)")
    parser.add_argument("--list-topics", action="store_true",
                        help="List all active gz topics and exit")
    args = parser.parse_args()

    if args.list_topics:
        list_topics()
    elif args.once:
        monitor_once()
    else:
        monitor_continuous(interval=args.interval)
