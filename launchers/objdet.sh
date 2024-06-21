#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

source /code/catkin_ws/devel/setup.bash --extend
#source /code/submission_ws/devel/setup.bash --extend
#source /code/solution/devel/setup.bash --extend

# Launch object detection node in background
dt-exec roslaunch --wait object_detection object_detection_node.launch veh:=$VEHICLE_NAME &
pid1=$!

# Launch modcon node in background
rosrun my_package modcon.py &
pid2=$!

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for both processes to end
wait $pid1
wait $pid2

# wait for app to end
dt-launchfile-join

