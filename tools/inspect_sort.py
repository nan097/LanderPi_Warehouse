import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
rclpy.init()
n = Node('inspect_costmaps')
data = {}
for (kind, topic, key) in [(OccupancyGrid, '/global_costmap/costmap', 'map'), (Odometry, '/odom', 'odom'), (LaserScan, '/scan', 'scan')]:
    n.create_subscription(kind, topic, lambda m, k=key: data.update({k: m}), qos_profile_sensor_data)
for _ in range(120):
    rclpy.spin_once(n, timeout_sec=0.1)
    if len(data) == 3:
        break
print('received', list(data))
if 'odom' in data:
    print('odom', data['odom'].pose.pose)
if 'scan' in data:
    s = data['scan']
    print('close rays', [(round(s.angle_min + i * s.angle_increment, 2), round(r, 3)) for (i, r) in enumerate(s.ranges) if math.isfinite(r) and r < 0.3][::4])
if 'map' in data:
    m = data['map']
    o = m.info.origin.position
    cells = []
    for (i, v) in enumerate(m.data):
        x = o.x + (i % m.info.width + 0.5) * m.info.resolution
        y = o.y + (i // m.info.width + 0.5) * m.info.resolution
        if -0.1 < x < 0.8 and abs(y) < 0.25 and (v >= 99):
            cells.append((round(x, 3), round(y, 3), v))
    print('lethal near pick', cells)
n.destroy_node()
rclpy.shutdown()
