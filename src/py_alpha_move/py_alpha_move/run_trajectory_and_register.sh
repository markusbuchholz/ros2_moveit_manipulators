#!/bin/bash

./register_joint_states.sh &

REGISTER_PID=$!
echo "Started register_joint_states.sh with PID ${REGISTER_PID}"

sleep 1


read -r -d '' WAYPOINTS << 'EOF'
  Step 1: [3.92701957 0.91591986 2.2        1.        ]
  Step 2: [3.99216136 1.19699727 2.2        1.        ]
  Step 3: [3.8367569  1.37676881 2.12477126 1.10397032]
  Step 4: [3.65118338 1.54268248 2.0392518  1.22216298]
  Step 5: [3.64289692 1.79075337 1.95291129 1.33504266]
  Step 6: [3.52922173 1.91929021 1.88453666 1.48008751]
  Step 7: [3.33213339 1.93265804 1.74769795 1.57180879]
  Step 8: [3.16342297 1.95021467 1.5143215  1.5683518 ]
  Step 9: [3.02100556 1.92496609 1.51895287 1.47023242]
  Step 10: [2.89891753 1.86662118 1.7076084  1.29892148]
  Step 11: [2.73806749 1.76950001 1.87472139 1.22942898]
  Step 12: [2.62119454 1.57032886 1.91860693 1.16067252]
  Step 13: [2.57717896 1.32868837 1.93097213 1.05046585]
  Step 14: [2.57812041 1.08391359 2.08218767 1.07205858]
  Step 15: [2.5        0.92372815 2.2        1.        ]
EOF

while IFS= read -r line; do
  [[ -z "$line" ]] && continue

  echo "Processing: $line"

  numbers=$(echo "$line" | sed -n 's/.*\[\(.*\)\].*/\1/p')

  read -ra vals <<< "$numbers"
  if [[ ${#vals[@]} -ne 4 ]]; then
    echo "Warning: Expected 4 numbers but got ${#vals[@]} in line: $line"
    continue
  fi

  reversed=()
  for (( i=${#vals[@]}-1; i>=0; i-- )); do
    reversed+=( "${vals[i]}" )
  done

  positions="0.0"
  for num in "${reversed[@]}"; do
    positions+=", ${num}"
  done

  echo "Publishing waypoint with positions: [${positions}]"

  message="joint_names:
- 'axis_a'
- 'axis_b'
- 'axis_c'
- 'axis_d'
- 'axis_e'
points:
- positions: [${positions}]
  time_from_start:
    sec: 2
    nanosec: 0"

  ros2 topic pub /xsubsea/joint_trajectory trajectory_msgs/msg/JointTrajectory "$message" -1

  sleep 1

done <<< "$WAYPOINTS"

echo "Terminating register_joint_states.sh (PID ${REGISTER_PID})"
kill ${REGISTER_PID}
