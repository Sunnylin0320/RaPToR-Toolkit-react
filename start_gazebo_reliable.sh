#!/bin/bash
MAX_ATTEMPTS=10
POLL_INTERVAL=2
MAX_POLL_TIME=40
WORLD="${1:-depot}"
# Default spawn coordinates differ per world — depot's centre is safe,
# but maze's centre is inside a wall cluster, so it needs a different
# default spawn point, verified manually against maze.sdf's obstacle
# coordinates.
if [ "$WORLD" = "maze" ]; then
    DEFAULT_X=2.0
    DEFAULT_Y=1.0
else
    DEFAULT_X=0.0
    DEFAULT_Y=0.0
fi

SPAWN_X="${2:-$DEFAULT_X}"
SPAWN_Y="${3:-$DEFAULT_Y}"

export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash

for ATTEMPT in $(seq 1 $MAX_ATTEMPTS); do
    echo "Using world: $WORLD, spawn position: ($SPAWN_X, $SPAWN_Y)"
    echo "=== Attempt $ATTEMPT of $MAX_ATTEMPTS ==="

    pkill -9 -f ign 2>/dev/null
    pkill -9 -f gz 2>/dev/null
    pkill -9 -f rviz 2>/dev/null
    sleep 3

    ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py \
    world:="$WORLD" \
    x:="$SPAWN_X" \
    y:="$SPAWN_Y" \
    ign_args:="--render-engine ogre" > /tmp/gazebo_attempt_$ATTEMPT.log 2>&1 &
    LAUNCH_PID=$!

    echo "Waiting for world control service to appear..."
    SERVICE=""
    for i in $(seq 1 30); do
        SERVICE=$(ign service -l 2>/dev/null | grep -m1 "/world/.*/control")
        if [ -n "$SERVICE" ]; then
            break
        fi
        sleep 1
    done

    if [ -z "$SERVICE" ]; then
        echo "World control service never appeared. Retrying..."
        kill -9 $LAUNCH_PID 2>/dev/null
        continue
    fi

    echo "Found service: $SERVICE. Waiting 3s before unpausing..."
    sleep 3

    echo "Sending unpause command..."
    ign service -s "$SERVICE" \
        --reqtype ignition.msgs.WorldControl \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req 'pause: false' > /dev/null

    echo "Polling controller status (up to ${MAX_POLL_TIME}s)..."
    ELAPSED=0
    SUCCESS=false

    while [ $ELAPSED -lt $MAX_POLL_TIME ]; do
        sleep $POLL_INTERVAL
        ELAPSED=$((ELAPSED + POLL_INTERVAL))

        STATUS=$(ros2 control list_controllers 2>/dev/null)

        JSB_OK=false
        DDC_OK=false

        if echo "$STATUS" | grep "joint_state_broadcaster" | grep -q "active" && \
           ! echo "$STATUS" | grep "joint_state_broadcaster" | grep -q "inactive"; then
            JSB_OK=true
        fi

        if echo "$STATUS" | grep "diffdrive_controller" | grep -q "active" && \
           ! echo "$STATUS" | grep "diffdrive_controller" | grep -q "inactive"; then
            DDC_OK=true
        fi

        echo "joint_state_broadcaster active: $JSB_OK | diffdrive_controller active: $DDC_OK"

        if [ "$JSB_OK" = true ] && [ "$DDC_OK" = true ]; then
            SUCCESS=true
            break
        fi

        if ! kill -0 $LAUNCH_PID 2>/dev/null; then
            echo "Launch process exited early."
            break
        fi
    done

    if [ "$SUCCESS" = true ]; then
        echo "SUCCESS after ${ELAPSED}s: Both controllers are active."
        echo ""
        echo "Gazebo is running (PID $LAUNCH_PID). Leave this terminal open."
        wait $LAUNCH_PID
        exit 0
    else
        echo "FAILED after ${ELAPSED}s."
        echo "Retrying..."
        kill -9 $LAUNCH_PID 2>/dev/null
    fi
done

echo "Gave up after $MAX_ATTEMPTS attempts."
exit 1
