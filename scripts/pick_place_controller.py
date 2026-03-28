#!/usr/bin/env python3
"""
UR5 Pick-and-Place State Machine Controller
============================================
Controls UR5 in Gazebo via gz-transport topics using fixed joint configurations.

State flow:
  IDLE → HOME → ABOVE_OBJECT → APPROACH_OBJECT → GRASP →
  LIFT → ABOVE_PLACE → LOWER_TO_PLACE → RELEASE → RETREAT → HOME → DONE

Requirements:
  conda activate gz_run
  pip install gz-transport (or use gz Python bindings)
  
Usage:
  python pick_place_controller.py [--cycle] [--verbose]
"""

import sys
import os
import time
import math
import argparse
import subprocess
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, Optional, List
import shutil

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

ROBOT_MODEL  = "ur5_robot"
TOPIC_PREFIX = f"/model/{ROBOT_MODEL}/joint"

# Topic template:  /model/ur5_robot/joint/<joint_name>/cmd_pos
JOINT_TOPICS = {
    "shoulder_pan_joint":  f"{TOPIC_PREFIX}/shoulder_pan_joint/cmd_pos",
    "shoulder_lift_joint": f"{TOPIC_PREFIX}/shoulder_lift_joint/cmd_pos",
    "elbow_pan_joint":     f"{TOPIC_PREFIX}/elbow_pan_joint/cmd_pos",
    "elbow_lift_joint":    f"{TOPIC_PREFIX}/elbow_lift_joint/cmd_pos",
    "wrist_1_joint":       f"{TOPIC_PREFIX}/wrist_1_joint/cmd_pos",
    "wrist_2_joint":       f"{TOPIC_PREFIX}/wrist_2_joint/cmd_pos",
    "left_finger_joint":   f"{TOPIC_PREFIX}/left_finger_joint/cmd_pos",
    "right_finger_joint":  f"{TOPIC_PREFIX}/right_finger_joint/cmd_pos",
}

# ──────────────────────────────────────────────────────────────────────────────
# JOINT CONFIGURATIONS  (all angles in radians, gripper in meters)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class JointConfig:
    """Named joint configuration (pose)."""
    name: str
    description: str
    joints: Dict[str, float]
    # How long to hold this pose before considering it "reached"
    hold_duration: float = 2.0
    # Gripper-specific hold (close/open needs extra settle time)
    gripper_settle: float = 0.5


JOINT_CONFIGS: Dict[str, JointConfig] = {
    "home": JointConfig(
        name="home",
        description="Safe upright home position",
        joints={
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     0.0,
            "wrist_1_joint":        0.0,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    0.01,
            "right_finger_joint":   0.01,
        },
        hold_duration=1.5,
    ),
    "above_object": JointConfig(
        name="above_object",
        description="Above pick object (~15cm hover)",
        joints={
            "shoulder_pan_joint":  0,
            "shoulder_lift_joint": 0.0,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.5708,
            "wrist_1_joint":        1.5708,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    0.01,
            "right_finger_joint":   0.01,
        },
        hold_duration=1.0,
    ),
    "approach_object": JointConfig(
        name="approach_object",
        description="Lowered to pick object level",
        joints={
            "shoulder_pan_joint":  0,
            "shoulder_lift_joint": 0.4,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.88,
            "wrist_1_joint":        0.95,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    0.01,
            "right_finger_joint":   0.01,
        },
        hold_duration=1.0,
    ),
    "grasp": JointConfig(
        name="grasp",
        description="Close gripper on object",
        joints={
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.4,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.88,
            "wrist_1_joint":        0.95,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    -0.02,
            "right_finger_joint":   -0.02,
        },
        hold_duration=1.5,
        gripper_settle=1.0,
    ),
    "lift": JointConfig(
        name="lift",
        description="Raise object clear of table",
        joints={
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.5708,
            "wrist_1_joint":        1.5708,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    -0.02,
            "right_finger_joint":   -0.02,
        },
        hold_duration=1.5,
    ),
    "above_place": JointConfig(
        name="above_place",
        description="Above place zone",
        joints={
            "shoulder_pan_joint":  1.5708,
            "shoulder_lift_joint": 0.0,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.5708,
            "wrist_1_joint":       1.5708,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    -0.02,
            "right_finger_joint":   -0.02,
        },
        hold_duration=2.0,
    ),
    "lower_to_place": JointConfig(
        name="lower_to_place",
        description="Lowered to place surface",
        joints={
            "shoulder_pan_joint":  1.5708,
            "shoulder_lift_joint": 0.4,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.88,
            "wrist_1_joint":        0.8,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    -0.02,
            "right_finger_joint":   -0.02,
        },
        hold_duration=2.0,
    ),
    "release": JointConfig(
        name="release",
        description="Open gripper to release object",
        joints={
            "shoulder_pan_joint":  1.5708,
            "shoulder_lift_joint": 0.4,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     1.88,
            "wrist_1_joint":        0.8,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    0.01,
            "right_finger_joint":   0.01,
        },
        hold_duration=1.5,
        gripper_settle=1.0,
    ),
    "retreat": JointConfig(
        name="retreat",
        description="Raise arm after placing",
        joints={
            "shoulder_pan_joint":  0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_pan_joint":      0.0,
            "elbow_lift_joint":     0.0,
            "wrist_1_joint":       0.0,
            "wrist_2_joint":        0.0,
            "left_finger_joint":    0.01,
            "right_finger_joint":   0.01,
        },
        hold_duration=2.0,
    ),
}

# ──────────────────────────────────────────────────────────────────────────────
# STATE MACHINE
# ──────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE           = auto()
    HOME           = auto()
    ABOVE_OBJECT   = auto()
    APPROACH       = auto()
    GRASP          = auto()
    LIFT           = auto()
    ABOVE_PLACE    = auto()
    LOWER_TO_PLACE = auto()
    RELEASE        = auto()
    RETREAT        = auto()
    RETURN_HOME    = auto()
    DONE           = auto()
    ERROR          = auto()


# Ordered state sequence (maps State → config key)
STATE_SEQUENCE = [
    (State.HOME,           "home"),
    (State.ABOVE_OBJECT,   "above_object"),
    (State.APPROACH,       "approach_object"),
    (State.GRASP,          "grasp"),
    (State.LIFT,           "lift"),
    (State.ABOVE_PLACE,    "above_place"),
    (State.LOWER_TO_PLACE, "lower_to_place"),
    (State.RELEASE,        "release"),
    (State.RETREAT,        "retreat"),
    (State.RETURN_HOME,    "home"),
    (State.DONE,           None),
]



# ──────────────────────────────────────────────────────────────────────────────
# EASING FUNCTIONS
# ──────────────────────────────────────────────────────────────────────────────

def ease_linear(t: float) -> float:
    return t

def ease_in_out_quad(t: float) -> float:
    """Smooth acceleration + deceleration (quadratic)."""
    if t < 0.5:
        return 2 * t * t
    return -1 + (4 - 2 * t) * t

def ease_in_out_sine(t: float) -> float:
    """Very smooth sinusoidal ease."""
    return -(math.cos(math.pi * t) - 1) / 2

def ease_in_out_cubic(t: float) -> float:
    """Cubic ease (more aggressive acceleration)."""
    if t < 0.5:
        return 4 * t * t * t
    p = 2 * t - 2
    return (p * p * p + 2) / 2

# Profile registry
EASING_PROFILES = {
    "linear":       ease_linear,
    "smooth":       ease_in_out_quad,
    "sine":         ease_in_out_sine,
    "cubic":        ease_in_out_cubic,
}
# ──────────────────────────────────────────────────────────────────────────────
# TRANSPORT LAYER  (wraps gz topic pub calls)
# ──────────────────────────────────────────────────────────────────────────────

class GzTransport:
    """
    Publishes joint commands via 'gz topic -t <topic> -m gz.msgs.Double -p value:<val>'
    This uses the gz CLI which is available in the gz_run conda environment.
    """

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self._pub_cache: Dict[str, subprocess.Popen] = {}

    def publish_double(self, topic: str, value: float) -> bool:
        """Publish a gz.msgs.Double to a topic using the gz CLI."""
        gz_path = shutil.which("gz") 
        cmd = [
            gz_path, "topic",
            "-t", topic,
            "-m", "gz.msgs.Double",
            "-p", f"data: {value:.6f}",
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=3.0,
                env=os.environ.copy()
            )
            if self.verbose:
                print(f"  [gz] {topic} → {value:.4f}  (rc={result.returncode})")
            if result.returncode != 0:
                print(f"[DEBUG] double 執行失敗，Return Code: {result.returncode}")
                print(f"[DEBUG] 標準輸出 (stdout): {result.stdout.strip()}")
                print(f"[DEBUG] 錯誤輸出 (stderr): {result.stderr.strip()}")
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            print(f"  [WARN] Timeout publishing to {topic}")
            return False
        except FileNotFoundError:
            print(f"  [ERROR] 'gz' binary not found. Is gz_run conda env active?")
            return False

    def send_joint_config(self, config: JointConfig) -> bool:
        """Send all joint positions for a named configuration."""
        success = True
        for joint_name, position in config.joints.items():
            topic = JOINT_TOPICS.get(joint_name)
            if topic is None:
                print(f"  [WARN] Unknown joint: {joint_name}")
                continue
            ok = self.publish_double(topic, position)
            success = success and ok
        return success

    def check_gz_available(self) -> bool:


        gz_executable = shutil.which("gz")
        if not gz_executable:
            print("[DEBUG] 系統完全找不到 gz 的路徑")
            return False

        try:
            r = subprocess.run(
                [gz_executable, "help"],
                capture_output=True, 
                text=True, 
                timeout=5,
                env=os.environ.copy()
            )
            if r.returncode != 0:
                print(f"[DEBUG] gz 執行失敗，Return Code: {r.returncode}")
                print(f"[DEBUG] 標準輸出 (stdout): {r.stdout.strip()}")
                print(f"[DEBUG] 錯誤輸出 (stderr): {r.stderr.strip()}")
            
            return True
            
        except Exception as e:
            print(f"[DEBUG] 執行過程發生異常: {str(e)}")
            return False

class GripperController:
    def __init__(self, model_name="ur5_robot"):
        self.model_name = model_name
        # 定義萬能膠的兩個控制頻道
        self.attach_topic = f"/model/{model_name}/detachable_joint/attach"
        self.detach_topic = f"/model/{model_name}/detachable_joint/detach"

    def _send_empty(self, topic: str):
        cmd = [
            "gz", "topic",
            "-t", topic,
            "-m", "gz.msgs.Empty",
            "-p", ""          # ← 空字串，不是 " "（空白）
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
        return result.returncode == 0

    def attach_object(self):
        print(" [Action] 啟動萬能膠 (Attach)...")
        return self._send_empty(self.attach_topic)

    def detach_object(self):
        print(" [Action] 解除萬能膠 (Detach)...")
        return self._send_empty(self.detach_topic)

    def initialize(self):
        """程式啟動時立刻 detach，確保初始狀態是分離的"""
        print(" [Init] 確保 detachable joint 初始狀態為分離...")
        self.detach_object()
        time.sleep(0.5)
# ──────────────────────────────────────────────────────────────────────────────
# PICK-PLACE CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────


class PickPlaceController:
    """
    State machine that drives a UR5 through a complete pick-and-place cycle
    using fixed joint configurations.
    """

    def __init__(self, verbose: bool = False, cycle: bool = False):
        self.verbose = verbose
        self.cycle   = cycle
        self.transport = GzTransport(verbose=verbose)
        self.gripper = GripperController()
        self.current_state = State.IDLE
        self.cycle_count   = 0
        self._running      = True

    # ─── helpers ──────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")

    def _separator(self):
        print("─" * 60)

    def _wait(self, duration: float, reason: str = ""):
        """Sleep with progress dots."""
        if reason and self.verbose:
            print(f"  ⏳  waiting {duration:.1f}s {reason}", end="", flush=True)
        interval = 0.2
        steps = max(1, int(duration / interval))
        for _ in range(steps):
            if not self._running:
                break
            time.sleep(interval)
            if reason and self.verbose:
                print(".", end="", flush=True)
        if reason and self.verbose:
            print(" ✓")
    def set_sim_run(self, run=True, world_name="pick_object"):
        """
        控制 Gazebo 模擬的 Play/Pause
        run=True  -> 開始運行
        run=False -> 暫停
        """
        state = "false" if run else "true"
        cmd = [
            "gz", "service",
            "-s", f"/world/{world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--req", f"pause: {state}"
        ]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            print(f"  [Sim] 狀態設定為: {'運行中' if run else '已暫停'}")
        except Exception as e:
            print(f"  [Error] 無法控制模擬狀態: {e}")

    # ─── state machine execution ───────────────────────────────────────────────

    def _execute_state(self, state: State, config_key: Optional[str]) -> bool:
        """Send joint config and wait for settle time."""
        if config_key is None:
            return True  # DONE state has no config

        config = JOINT_CONFIGS.get(config_key)
        if config is None:
            self._log(f"[ERROR] Unknown config key: {config_key}")
            return False

        # Pretty state display
        self._separator()
        emoji_map = {
            State.HOME:           "🏠",
            State.ABOVE_OBJECT:   "🔍",
            State.APPROACH:       "⬇️ ",
            State.GRASP:          "🤏",
            State.LIFT:           "⬆️ ",
            State.ABOVE_PLACE:    "✈️ ",
            State.LOWER_TO_PLACE: "⬇️ ",
            State.RELEASE:        "🖐️ ",
            State.RETREAT:        "↩️ ",
            State.RETURN_HOME:    "🏠",
        }
        emoji = emoji_map.get(state, "→")
        self._log(f"{emoji}  State: {state.name}  |  {config.description}")

        if self.verbose:
            for jname, jval in config.joints.items():
                print(f"         {jname:25s} = {jval:+.4f}")

        
        if state == State.RELEASE:
            self.gripper.detach_object()
            self._wait(0.5, reason="detaching object")

        # 2. 發送關節指令 (這會讓夾爪物理性地閉合或張開)
        ok = self.transport.send_joint_config(config)
        if not ok:
            self._log(f"[WARN] Some joint commands failed for {config_key}")

        # 3. 等待關節動作完成 (Settle time)
        total_wait = config.hold_duration + config.gripper_settle
        self._wait(total_wait, reason="settling")

        # 4. 處理 GRASP 狀態後的動作 (夾緊後，啟動萬能膠)
        if state == State.GRASP:
            self.gripper.attach_object()
            self._log(" [DEBUG] 萬能膠已啟動")

        return True

    def run(self):
        """Execute the full pick-and-place state machine."""

        self._log("=" * 60)
        self._log("  UR5 Pick-and-Place State Machine Controller")
        self._log("=" * 60)

        
        # Pre-flight check
        if not self.transport.check_gz_available():
            self._log("[ERROR] 'gz' command not found!")
            self._log("  Make sure gz_run conda env is active:")
            self._log("    conda activate gz_run")
            self._log("  And Gazebo simulation is running:")
            self._log("    gz sim worlds/pick_place.sdf")
            sys.exit(1)

        self._log("✅  gz binary found")
        self._log("🔧  初始化 detachable joint（強制分離）...")
        self.gripper.initialize()
        self._log("🚀  Starting pick-and-place sequence...")

        try:
            while self._running:
                self.cycle_count += 1
                if self.cycle:
                    self._log(f"\n{'='*60}")
                    self._log(f"  CYCLE #{self.cycle_count}")
                    self._log(f"{'='*60}")

                for state, config_key in STATE_SEQUENCE:
                    if not self._running:
                        break
                    self.current_state = state

                    if state == State.DONE:
                        self._separator()
                        self._log(f"🎉  Pick-and-place cycle complete!")
                        if self.cycle:
                            self._log(f"   Repeating in 3 seconds... (Ctrl+C to stop)")
                            self._wait(3.0)
                        break

                    ok = self._execute_state(state, config_key)
                    if not ok:
                        self._log(f"[ERROR] Failed in state {state.name}")
                        self.current_state = State.ERROR
                        sys.exit(1)

                if not self.cycle:
                    break

        except KeyboardInterrupt:
            self._log("\n⚠️   Interrupted by user")
        finally:
            self._separator()
            self._log("Controller stopped. Robot holds last joint positions.")


# ──────────────────────────────────────────────────────────────────────────────
# INTERACTIVE DEMO MODE  (single step)
# ──────────────────────────────────────────────────────────────────────────────

class InteractiveController:
    """Step through states one at a time, waiting for user input."""

    def __init__(self, verbose: bool = False):
        self.transport = GzTransport(verbose=verbose)
        self.verbose   = verbose
        self.gripper = GripperController()
        self.state_idx = 0

    def run(self):
        print("\n" + "="*60)
        print("  UR5 Pick-and-Place  |  INTERACTIVE MODE")
        print("  Press ENTER to advance, 'q' to quit")
        print("="*60)

        if not self.transport.check_gz_available():
            print("[ERROR] gz binary not found. Activate conda env: gz_run")
            sys.exit(1)

        for i, (state, config_key) in enumerate(STATE_SEQUENCE):
            if config_key is None:
                print("\n🎉  Sequence complete!")
                break

            config = JOINT_CONFIGS[config_key]
            print(f"\nNext: [{i+1}/{len(STATE_SEQUENCE)-1}]  {state.name}")
            print(f"      {config.description}")
            cmd = input("  [ENTER=execute, 'q'=quit]: ").strip().lower()
            if cmd == "q":
                break

            if state == State.RELEASE:
                self.gripper.detach_object()

            print(f"  Sending joint config '{config_key}'...")
            self.transport.send_joint_config(config)
            print(f"  Waiting {config.hold_duration}s to settle...")
            time.sleep(config.hold_duration + config.gripper_settle)
            
            # 夾緊後再啟動萬能膠
            if state == State.GRASP:
                self.gripper.attach_object()
                
            print("  ✓ Done")

        print("\nInteractive session ended.")


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="UR5 Pick-and-Place State Machine Controller for Gazebo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run single automatic cycle
  python pick_place_controller.py

  # Run continuously (loop)
  python pick_place_controller.py --cycle

  # Step through interactively
  python pick_place_controller.py --interactive

  # Verbose output (show all joint values)
  python pick_place_controller.py --verbose
        """,
    )
    parser.add_argument("--cycle",       action="store_true",
                        help="Repeat the pick-and-place cycle indefinitely")
    parser.add_argument("--verbose",     action="store_true",
                        help="Show detailed joint angle output")
    parser.add_argument("--interactive", action="store_true",
                        help="Step through states one at a time (manual ENTER)")
    args = parser.parse_args()

    if args.interactive:
        ctrl = InteractiveController(verbose=args.verbose)
        ctrl.run()
    else:
        ctrl = PickPlaceController(verbose=args.verbose, cycle=args.cycle)
        ctrl.run()


if __name__ == "__main__":
    main()
