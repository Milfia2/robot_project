#!/usr/bin/env bash
# ============================================================
# UR5 Pick-and-Place Launch Script
# ============================================================
# Usage:
#   chmod +x launch.sh
#   ./launch.sh                    # auto mode (full sequence)
#   ./launch.sh --interactive      # step-by-step mode
#   ./launch.sh --cycle            # repeat indefinitely
#   ./launch.sh --gz-only          # launch Gazebo only (no controller)
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$SCRIPT_DIR/worlds/pick_place.sdf"
CONTROLLER="$SCRIPT_DIR/scripts/pick_place_controller.py"
CONTROLLER_ARGS=""
GZ_ONLY=false

# Parse args
for arg in "$@"; do
    case $arg in
        --interactive) CONTROLLER_ARGS="--interactive" ;;
        --cycle)       CONTROLLER_ARGS="--cycle" ;;
        --verbose)     CONTROLLER_ARGS="$CONTROLLER_ARGS --verbose" ;;
        --gz-only)     GZ_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [--interactive] [--cycle] [--verbose] [--gz-only]"
            exit 0
            ;;
    esac
done

# ─── Check conda env ──────────────────────────────────────────────────────────
if [[ -z "$CONDA_DEFAULT_ENV" ]]; then
    echo "⚠  No conda env active. Attempting to activate gz_run..."
    source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
    conda activate gz_run 2>/dev/null || {
        echo "ERROR: Could not activate gz_run. Run manually:"
        echo "  conda activate gz_run"
        exit 1
    }
fi

echo "✅  Conda env: $CONDA_DEFAULT_ENV"

# ─── Export model path ────────────────────────────────────────────────────────
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
echo "✅  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

# ─── Launch Gazebo ────────────────────────────────────────────────────────────
echo ""
echo "🚀  Launching Gazebo..."
echo "    World: $WORLD_FILE"
echo ""

if $GZ_ONLY; then
    exec gz sim "$WORLD_FILE" -r
fi

# Launch Gazebo in background
gz sim "$WORLD_FILE" -r &
GZ_PID=$!
echo "✅  Gazebo PID: $GZ_PID"

# Wait for Gazebo to start
echo "⏳  Waiting for Gazebo to initialize (5 seconds)..."
sleep 5

# Check if Gazebo is still running
if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "❌  Gazebo process ended unexpectedly"
    exit 1
fi
echo "✅  Gazebo is running"

# ─── Launch Controller ────────────────────────────────────────────────────────
echo ""
echo "🤖  Launching pick-and-place controller..."
echo "    Args: $CONTROLLER_ARGS"
echo ""

python "$CONTROLLER" $CONTROLLER_ARGS

# Cleanup
echo ""
echo "🛑  Stopping Gazebo..."
kill $GZ_PID 2>/dev/null || true
wait $GZ_PID 2>/dev/null || true
echo "✅  Done"
