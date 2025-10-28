#!/bin/bash

# Centerline Agreement Analysis Launch Script
# This script launches both windrow detection nodes and the agreement analyzer

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Centerline Agreement Analysis${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if nodes exist
if ! ros2 pkg executables windrow_centerline_node | grep -q windrow_centerline_node; then
    echo -e "${RED}ERROR: windrow_centerline_node not found!${NC}"
    echo "Please build the workspace first:"
    echo "  cd ~/ros2_ws && colcon build --packages-select windrow_centerline_node"
    exit 1
fi

# Parse arguments
OUTPUT_DIR="${1:-./centerline_analysis}"
TOP_N="${2:-3}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  Output Directory: $OUTPUT_DIR"
echo "  Top N Best/Worst: $TOP_N"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}Starting nodes in separate terminals...${NC}"
echo ""
echo -e "${YELLOW}Instructions:${NC}"
echo "  1. Three terminal windows will open"
echo "  2. Wait for all nodes to initialize"
echo "  3. Play your bag file: ros2 bag play <your_bag.db3>"
echo "  4. Press Ctrl+C in the analyzer terminal when done"
echo "  5. Check results in: $OUTPUT_DIR"
echo ""
read -p "Press Enter to continue..."

# Function to check if gnome-terminal is available
if command -v gnome-terminal &> /dev/null; then
    TERM_CMD="gnome-terminal"
elif command -v xterm &> /dev/null; then
    TERM_CMD="xterm -e"
elif command -v konsole &> /dev/null; then
    TERM_CMD="konsole -e"
else
    echo -e "${RED}ERROR: No suitable terminal emulator found${NC}"
    echo "Please install gnome-terminal, xterm, or konsole"
    exit 1
fi

# Launch ZED centerline node
echo -e "${GREEN}[1/3] Launching ZED centerline detection...${NC}"
gnome-terminal --title="ZED Centerline" -- bash -c "
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    echo -e '${GREEN}Starting ZED centerline detection...${NC}'
    ros2 run windrow_centerline_node windrow_centerline_centroid_node --ros-args \
      -r __ns:=/zed \
      -p input_topic:=/zed/zed_node/point_cloud/cloud_registered/filtered \
      -p output_centerline_topic:=/zed/windrow_centerline \
      -p target_frame:=base_link
    exec bash
" &

sleep 2

# Launch Ouster centerline node
echo -e "${GREEN}[2/3] Launching Ouster centerline detection...${NC}"
gnome-terminal --title="Ouster Centerline" -- bash -c "
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    echo -e '${GREEN}Starting Ouster centerline detection...${NC}'
    ros2 run windrow_centerline_node windrow_centerline_centroid_node --ros-args \
      -r __ns:=/ouster \
      -p input_topic:=/ouster/points \
      -p output_centerline_topic:=/ouster/windrow_centerline \
      -p target_frame:=base_link
    exec bash
" &

sleep 2

# Launch agreement analyzer
echo -e "${GREEN}[3/3] Launching agreement analyzer...${NC}"
gnome-terminal --title="Agreement Analyzer" -- bash -c "
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws/src/windrow_centerline_node/eval
    echo -e '${GREEN}Starting agreement analyzer...${NC}'
    echo -e '${YELLOW}Output will be saved to: $OUTPUT_DIR${NC}'
    echo -e '${YELLOW}Press Ctrl+C when done to generate summary${NC}'
    python3 centerline_agreement_analyzer.py --ros-args \
      -p output_dir:=$OUTPUT_DIR \
      -p top_n_best:=$TOP_N \
      -p top_n_worst:=$TOP_N
    exec bash
" &

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All nodes launched!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Wait ~5 seconds for nodes to initialize"
echo "  2. Play your bag file in a new terminal:"
echo "     ${GREEN}ros2 bag play <your_30min_dataset.db3>${NC}"
echo "  3. Monitor progress in the analyzer terminal"
echo "  4. Press Ctrl+C in analyzer terminal when done"
echo "  5. View results in: ${GREEN}$OUTPUT_DIR${NC}"
echo ""

