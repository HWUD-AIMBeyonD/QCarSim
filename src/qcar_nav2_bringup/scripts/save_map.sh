#!/bin/bash
# Save the current SLAM map.
# Usage: ./save_map.sh [map_name] [output_dir]
#   map_name   - base filename for the .pgm/.yaml (default: qcar_map)
#   output_dir - directory to write to (default: current directory)

MAP_NAME="${1:-qcar_map}"
OUTPUT_DIR="${2:-.}"

echo "Saving map as '${OUTPUT_DIR}/${MAP_NAME}'..."
ros2 run nav2_map_server map_saver_cli -f "${OUTPUT_DIR}/${MAP_NAME}" --ros-args -p use_sim_time:=true
echo "Done. Files: ${OUTPUT_DIR}/${MAP_NAME}.pgm, ${OUTPUT_DIR}/${MAP_NAME}.yaml"
