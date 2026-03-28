#!/usr/bin/env python3
"""
Manual Joint Commander
======================
Send individual joint position commands to the UR5 robot.
Useful for calibrating poses and debugging joint configurations.

Usage:
  python manual_joint_cmd.py --joint shoulder_pan_joint --value 1.57
  python manual_joint_cmd.py --pose home
  python manual_joint_cmd.py --pose above_object --verbose
  python manual_joint_cmd.py --list-poses
  python manual_joint_cmd.py --open-gripper
  python manual_joint_cmd.py --close-gripper
"""

import sys
import subprocess
import argparse

# Import from our controller
sys.path.insert(0, ".")
try:
    from pick_place_controller import JOINT_CONFIGS, JOINT_TOPICS, GzTransport
except ImportError:
    print("Run from scripts/ directory: cd scripts && python manual_joint_cmd.py")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Manual Joint Commander for UR5",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--joint",        help="Joint name (use with --value)")
    group.add_argument("--pose",         help="Named pose to send")
    group.add_argument("--list-poses",   action="store_true",
                       help="List all available poses")
    group.add_argument("--open-gripper",  action="store_true",
                       help="Open the gripper")
    group.add_argument("--close-gripper", action="store_true",
                       help="Close the gripper")

    parser.add_argument("--value",   type=float, help="Joint value (rad or m)")
    parser.add_argument("--verbose", action="store_true")

    args = parser.parse_args()
    transport = GzTransport(verbose=args.verbose)

    if not transport.check_gz_available():
        print("[ERROR] gz binary not found. Activate gz_run conda env.")
        sys.exit(1)

    if args.list_poses:
        print(f"\nAvailable poses ({len(JOINT_CONFIGS)}):")
        for name, cfg in JOINT_CONFIGS.items():
            print(f"  {name:<20}  {cfg.description}")
        return

    if args.joint:
        if args.value is None:
            print("[ERROR] --value required when using --joint")
            sys.exit(1)
        topic = JOINT_TOPICS.get(args.joint)
        if topic is None:
            print(f"[ERROR] Unknown joint: {args.joint}")
            print(f"Available joints: {list(JOINT_TOPICS.keys())}")
            sys.exit(1)
        print(f"Setting {args.joint} = {args.value:.4f}")
        ok = transport.publish_double(topic, args.value)
        print("✓ Sent" if ok else "✗ Failed")

    elif args.pose:
        config = JOINT_CONFIGS.get(args.pose)
        if config is None:
            print(f"[ERROR] Unknown pose: {args.pose}")
            print(f"Available: {list(JOINT_CONFIGS.keys())}")
            sys.exit(1)
        print(f"Sending pose '{args.pose}': {config.description}")
        ok = transport.send_joint_config(config)
        print("✓ All joints sent" if ok else "⚠ Some joints failed")

    elif args.open_gripper:
        print("Opening gripper...")
        transport.publish_double(JOINT_TOPICS["left_finger_joint"],  0.04)
        transport.publish_double(JOINT_TOPICS["right_finger_joint"], 0.04)
        print("✓ Gripper open command sent")

    elif args.close_gripper:
        print("Closing gripper...")
        transport.publish_double(JOINT_TOPICS["left_finger_joint"],  0.005)
        transport.publish_double(JOINT_TOPICS["right_finger_joint"], 0.005)
        print("✓ Gripper close command sent")


if __name__ == "__main__":
    main()
