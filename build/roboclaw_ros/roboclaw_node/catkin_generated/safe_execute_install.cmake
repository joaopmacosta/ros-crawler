execute_process(COMMAND "/home/jcosta/workspace/ROS/crawler/build/roboclaw_ros/roboclaw_node/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jcosta/workspace/ROS/crawler/build/roboclaw_ros/roboclaw_node/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
