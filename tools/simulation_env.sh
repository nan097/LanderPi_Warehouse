# Source from project root after /opt/ros/humble/setup.bash.
LANDERPI_LOCAL_PREFIX="$PWD/.deps/nav2_root/opt/ros/humble"
if [ -d "$LANDERPI_LOCAL_PREFIX" ]; then
 export AMENT_PREFIX_PATH="$LANDERPI_LOCAL_PREFIX:${AMENT_PREFIX_PATH:-}"
 export CMAKE_PREFIX_PATH="$LANDERPI_LOCAL_PREFIX:${CMAKE_PREFIX_PATH:-}"
 export LD_LIBRARY_PATH="$LANDERPI_LOCAL_PREFIX/lib:$LANDERPI_LOCAL_PREFIX/lib/x86_64-linux-gnu:$PWD/.deps/nav2_root/usr/lib:$PWD/.deps/nav2_root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
 export PYTHONPATH="$LANDERPI_LOCAL_PREFIX/local/lib/python3.10/dist-packages:$LANDERPI_LOCAL_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
 export PATH="$LANDERPI_LOCAL_PREFIX/bin:$PATH"
fi
export GAZEBO_PLUGIN_PATH="$PWD/install/landerpi_simulation/lib:${GAZEBO_PLUGIN_PATH:-}"

# UDP avoids stale Fast DDS shared-memory segments across interrupted simulation runs.
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/landerpi_simulation/config/fastdds_udp.xml"
export ROS_LOCALHOST_ONLY=1

# Prefer Cyclone DDS for stable long-running Nav2 TF subscriptions.
if [ -f "$LANDERPI_LOCAL_PREFIX/lib/librmw_cyclonedds_cpp.so" ]; then
 export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
 export CYCLONEDDS_URI="file://$PWD/src/landerpi_simulation/config/cyclonedds.xml"
fi
