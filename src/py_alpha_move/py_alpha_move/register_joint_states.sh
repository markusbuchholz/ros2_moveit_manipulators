#!/bin/bash

LOGFILE="joint_states_log.txt"

START_TIME=$(date +%s.%N)

echo "ElapsedTime, Axis_A, Axis_B, Axis_C, Axis_D, Axis_E" > "$LOGFILE"

while true; do
    MSG=$(ros2 topic echo /joint_states --once 2>/dev/null)
    
    if [ -z "$MSG" ]; then
        echo "No message received. Is the topic /joint_states publishing?" >&2
        sleep 0.1
        continue
    fi

    CURRENT_TIME=$(date +%s.%N)
    
    ELAPSED_TIME=$(awk "BEGIN {printf \"%.6f\", $CURRENT_TIME - $START_TIME}")

    POSITIONS=$(echo "$MSG" | awk '/position:/{flag=1; next} /velocity:/{flag=0} flag' \
                | sed 's/^- *//g' | tr '\n' ',' | sed 's/,$//')
    
    echo "$ELAPSED_TIME, $POSITIONS" >> "$LOGFILE"
    
    sleep 0.1
done
