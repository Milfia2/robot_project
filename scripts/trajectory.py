#!/usr/bin/env python3
"""
Trajectory Interpolator
=======================
Provides smooth joint motion between configurations using
trapezoidal velocity profiles (no external deps beyond math).

Instead of jumping directly to target joint angles, this module
generates intermediate waypoints at a configurable rate so the
robot moves smoothly rather than teleporting.

Usage (standalone):
  from trajectory import TrajectoryExecutor
  exec = TrajectoryExecutor(transport, hz=50)
  exec.move_to(config_a, config_b, duration=2.0)
"""

import time
import math
import threading
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, field


# ──────────────────────────────────────────────────────────────────────────────
# DATA TYPES
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class JointWaypoint:
    """Single joint state snapshot."""
    joints: Dict[str, float]
    t: float = 0.0           # normalized time [0, 1]


@dataclass
class TrajectorySegment:
    """A linear move from one JointConfig to another."""
    start:    Dict[str, float]
    end:      Dict[str, float]
    duration: float               # seconds
    name:     str = ""


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
# TRAJECTORY GENERATOR
# ──────────────────────────────────────────────────────────────────────────────

class TrajectoryGenerator:
    """
    Generates interpolated joint positions along a trajectory segment.
    """

    def __init__(self, profile: str = "smooth"):
        self.ease_fn = EASING_PROFILES.get(profile, ease_in_out_quad)

    def interpolate(
        self,
        start: Dict[str, float],
        end:   Dict[str, float],
        t_norm: float,              # 0.0 → 1.0
    ) -> Dict[str, float]:
        """Interpolate joint positions at normalized time t_norm."""
        alpha = self.ease_fn(max(0.0, min(1.0, t_norm)))
        result = {}
        for joint in start:
            if joint in end:
                result[joint] = start[joint] + alpha * (end[joint] - start[joint])
            else:
                result[joint] = start[joint]
        return result

    def generate_waypoints(
        self,
        start:    Dict[str, float],
        end:      Dict[str, float],
        duration: float,
        hz:       float = 50.0,
    ) -> List[Tuple[float, Dict[str, float]]]:
        """
        Generate list of (timestamp, joint_dict) pairs.
        Returns waypoints spaced at 1/hz seconds.
        """
        dt = 1.0 / hz
        n_steps = max(1, int(duration * hz))
        waypoints = []

        for i in range(n_steps + 1):
            t_abs  = i * dt
            t_norm = min(1.0, t_abs / duration)
            joints = self.interpolate(start, end, t_norm)
            waypoints.append((t_abs, joints))

        return waypoints

    def max_joint_delta(
        self,
        start: Dict[str, float],
        end:   Dict[str, float],
    ) -> float:
        """Return the largest joint angle change across all joints."""
        return max(
            abs(end.get(j, 0) - start.get(j, 0))
            for j in start
        )

    def auto_duration(
        self,
        start:      Dict[str, float],
        end:        Dict[str, float],
        max_vel:    float = 1.0,    # rad/s (conservative)
        min_time:   float = 0.5,
    ) -> float:
        """
        Calculate minimum safe duration based on max joint velocity.
        """
        max_delta = self.max_joint_delta(start, end)
        needed = max_delta / max_vel
        return max(needed, min_time)


# ──────────────────────────────────────────────────────────────────────────────
# TRAJECTORY EXECUTOR
# ──────────────────────────────────────────────────────────────────────────────

class TrajectoryExecutor:
    """
    Drives joint motion by publishing interpolated commands at a fixed rate.
    Drop-in replacement for direct GzTransport.send_joint_config() calls.
    """

    def __init__(
        self,
        transport,                   # GzTransport instance
        hz:       float = 50.0,
        profile:  str   = "smooth",
        verbose:  bool  = False,
    ):
        self.transport  = transport
        self.hz         = hz
        self.dt         = 1.0 / hz
        self.generator  = TrajectoryGenerator(profile=profile)
        self.verbose    = verbose
        self._stop_flag = threading.Event()

        # Track current joint positions (for chaining moves)
        self._current: Dict[str, float] = {}

    def stop(self):
        """Signal any running trajectory to stop."""
        self._stop_flag.set()

    def move_to(
        self,
        start:        Dict[str, float],
        end:          Dict[str, float],
        duration:     Optional[float] = None,
        auto_duration: bool = True,
        on_progress:  Optional[Callable[[float], None]] = None,
    ) -> bool:
        """
        Execute a smooth move from start → end joint config.

        Args:
            start:         Starting joint positions (dict)
            end:           Target joint positions (dict)
            duration:      Time in seconds (auto-calculated if None)
            auto_duration: Use velocity-based auto timing
            on_progress:   Optional callback(0.0..1.0) for progress

        Returns:
            True if completed, False if interrupted
        """
        self._stop_flag.clear()

        if duration is None:
            if auto_duration:
                duration = self.generator.auto_duration(start, end)
            else:
                duration = 2.0

        if self.verbose:
            delta = self.generator.max_joint_delta(start, end)
            print(f"  [traj] {duration:.2f}s, max_Δ={math.degrees(delta):.1f}°, hz={self.hz}")

        waypoints = self.generator.generate_waypoints(start, end, duration, self.hz)
        t_start = time.perf_counter()

        for i, (t_rel, joints) in enumerate(waypoints):
            if self._stop_flag.is_set():
                return False

            # Send all joint commands for this waypoint
            for joint_name, value in joints.items():
                from pick_place_controller import JOINT_TOPICS
                topic = JOINT_TOPICS.get(joint_name)
                if topic:
                    self.transport.publish_double(topic, value)

            # Update internal state
            self._current = dict(joints)

            # Report progress
            progress = (i + 1) / len(waypoints)
            if on_progress:
                on_progress(progress)

            # Real-time pacing
            t_elapsed = time.perf_counter() - t_start
            t_target  = t_rel
            sleep_time = t_target - t_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return True

    def move_through(
        self,
        waypoint_configs: List[Dict[str, float]],
        durations:        Optional[List[float]] = None,
        label:            str = "",
    ) -> bool:
        """
        Execute a sequence of smooth moves through multiple waypoints.

        Args:
            waypoint_configs: List of joint dicts
            durations:        Per-segment durations (auto if None)
        """
        if len(waypoint_configs) < 2:
            return True

        n_segments = len(waypoint_configs) - 1
        if durations is None:
            durations = [None] * n_segments

        if self.verbose and label:
            print(f"  [traj] Multi-segment trajectory: '{label}' ({n_segments} segments)")

        for i in range(n_segments):
            if self._stop_flag.is_set():
                return False
            ok = self.move_to(
                start    = waypoint_configs[i],
                end      = waypoint_configs[i + 1],
                duration = durations[i],
            )
            if not ok:
                return False

        return True


# ──────────────────────────────────────────────────────────────────────────────
# SMOOTH PICK-PLACE CONTROLLER  (replaces instantaneous jumps)
# ──────────────────────────────────────────────────────────────────────────────

class SmoothPickPlaceController:
    """
    Drop-in replacement for PickPlaceController that uses smooth trajectories.
    Uses TrajectoryExecutor for interpolated motion instead of instant jumps.
    """

    def __init__(
        self,
        hz:      float = 50.0,
        profile: str   = "smooth",
        verbose: bool  = False,
        cycle:   bool  = False,
    ):
        import sys, os
        sys.path.insert(0, os.path.dirname(__file__))
        from pick_place_controller import (
            GzTransport, JOINT_CONFIGS, STATE_SEQUENCE, State
        )
        self.JOINT_CONFIGS  = JOINT_CONFIGS
        self.STATE_SEQUENCE = STATE_SEQUENCE
        self.State          = State

        self.transport = GzTransport(verbose=False)
        self.executor  = TrajectoryExecutor(self.transport, hz=hz,
                                            profile=profile, verbose=verbose)
        self.verbose = verbose
        self.cycle   = cycle
        self._running = True

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")

    def run(self):
        """Execute smooth pick-and-place sequence."""
        self._log("=" * 60)
        self._log("  UR5 Smooth Pick-and-Place Controller")
        self._log(f"  Profile: {self.executor.generator.ease_fn.__name__}, hz={self.executor.hz}")
        self._log("=" * 60)

        if not self.transport.check_gz_available():
            self._log("[ERROR] gz binary not found. Activate gz_run conda env.")
            return

        try:
            while self._running:
                prev_joints = None

                for state, config_key in self.STATE_SEQUENCE:
                    if not self._running:
                        break
                    if config_key is None:
                        self._log("🎉  Cycle complete!")
                        if self.cycle:
                            self._log("   Repeating in 2s...")
                            time.sleep(2.0)
                        break

                    config = self.JOINT_CONFIGS[config_key]

                    emoji_map = {
                        "home":           "🏠",
                        "above_object":   "🔍",
                        "approach_object":"⬇️ ",
                        "grasp":          "🤏",
                        "lift":           "⬆️ ",
                        "above_place":    "✈️ ",
                        "lower_to_place": "⬇️ ",
                        "release":        "🖐️ ",
                        "retreat":        "↩️ ",
                    }
                    emoji = emoji_map.get(config_key, "→")
                    self._log(f"{emoji}  {state.name}  →  {config.description}")

                    # Use previous joint state as start (smooth chaining)
                    start = prev_joints if prev_joints else config.joints

                    ok = self.executor.move_to(
                        start=start,
                        end=config.joints,
                        auto_duration=True,
                    )

                    if not ok:
                        self._log("[WARN] Trajectory interrupted")
                        break

                    prev_joints = dict(config.joints)

                    # Extra settle for gripper operations
                    if config.gripper_settle > 0:
                        time.sleep(config.gripper_settle)

                if not self.cycle:
                    break

        except KeyboardInterrupt:
            self._log("\n⚠️   Interrupted")
            self.executor.stop()


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Smooth trajectory pick-and-place")
    parser.add_argument("--hz",      type=float, default=50.0,  help="Control rate Hz")
    parser.add_argument("--profile", default="smooth",
                        choices=list(EASING_PROFILES.keys()),
                        help="Easing profile")
    parser.add_argument("--cycle",   action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    ctrl = SmoothPickPlaceController(
        hz=args.hz, profile=args.profile,
        verbose=args.verbose, cycle=args.cycle
    )
    ctrl.run()
