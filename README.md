colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

micrortps_agent