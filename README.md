colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

micrortps_agent -d /dev/ttyTHS0 -b 3000000