import rosbag
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
import rospy
import numpy
import eigenpy
import math
import sys
import os

os.rename(sys.argv[1], sys.argv[1] + '.orig')

ori_bag = rosbag.Bag(sys.argv[1] + '.orig')
new_bag = rosbag.Bag(sys.argv[1], 'w')

resolution = 1/180.*math.pi
stop_time = None


n = 0
print('parsing velodyne points...')
for topic, msg, t in ori_bag.read_messages(topics=['/carla/ego_vehicle/lidar/lidar1/point_cloud']):
    n += 1
    print('\r%d' % n),
    sys.stdout.flush()

    points_list = []
    for data in pcl2.read_points(msg):
        x = data[0]
        y = data[1]
        z = data[2]
        points_list.append([x, y, z])
    points_array = numpy.array(points_list).reshape(-1, 3)
    points_array = numpy.flip(points_array, axis=0)
    intensity = numpy.arange(points_array.shape[0]).reshape(-1, 1)
    points_array = numpy.append(points_array, intensity, axis=1)
    x = points_array[:, 0]
    y = points_array[:, 1]
    z = points_array[:, 2]
    deg = numpy.arctan2(z, numpy.sqrt(x**2 + y**2)).reshape(-1, 1)
    ring = numpy.zeros(deg.shape)
    last_ring = -1
    last_deg = float('-inf')
    for i in range(ring.size):
        if deg[i, 0] - last_deg > resolution:
            last_ring += 1
        last_deg = deg[i, 0]
        ring[i, 0] = last_ring
    if last_ring == 0:
        stop_time = t
        break
    points_array = numpy.append(points_array, ring, axis=1)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1),
              PointField('ring', 16, PointField.UINT16, 1)]
    new_msg = pcl2.create_cloud(msg.header, fields, points_array)
    new_bag.write(
        '/carla/ego_vehicle/lidar/lidar1/velodyne_points0', new_msg, t=t)
    distance = numpy.sqrt(x**2 + y**2 + z**2)
    noise = numpy.random.normal(0, 0.03)
    ratio = (distance+noise)/distance
    points_array[:, 0] = points_array[:, 0]*ratio
    points_array[:, 1] = points_array[:, 1]*ratio
    points_array[:, 2] = points_array[:, 2]*ratio
    new_msg = pcl2.create_cloud(msg.header, fields, points_array)
    new_bag.write(
        '/carla/ego_vehicle/lidar/lidar1/velodyne_points', new_msg, t=t)

n = 0
print('\nparsing odometry...')
origin = numpy.zeros([4, 4])
for topic, msg, t in ori_bag.read_messages(topics=['/carla/ego_vehicle/odometry']):
    n += 1
    print('\r%d' % n),
    sys.stdout.flush()

    if stop_time is not None and t > stop_time:
        break
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = eigenpy.Quaternion(qw, qx, qy, qz)
    odometry = numpy.eye(4)
    if numpy.array_equal(origin, numpy.zeros([4, 4])):
        origin[3, 3] = 1
        origin[:3, :3] = q.matrix()
        origin[0, 3] = x
        origin[1, 3] = y
        origin[2, 3] = z
    else:
        odometry[:3, :3] = q.matrix()
        odometry[0, 3] = x
        odometry[1, 3] = y
        odometry[2, 3] = z
        odometry = numpy.dot(numpy.linalg.inv(origin), odometry)
    x = odometry[0, 3]
    y = odometry[1, 3]
    z = odometry[2, 3]
    q = eigenpy.Quaternion(odometry[:3, :3])
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.x = q.x
    msg.pose.pose.orientation.y = q.y
    msg.pose.pose.orientation.z = q.z
    msg.pose.pose.orientation.w = q.w
    new_bag.write('/carla/ego_vehicle/odometry0', msg, t=t)

n = 0
print('\nparsing other...')
for topic, msg, t in ori_bag.read_messages(topics=None):
    if topic in ['/carla/ego_vehicle/lidar/lidar1/velodyne_points0',
                 '/carla/ego_vehicle/lidar/lidar1/velodyne_points',
                 '/carla/ego_vehicle/odometry0']:
        continue
    n += 1
    print('\r%d' % (n/10)),
    sys.stdout.flush()
    if stop_time is not None and t > stop_time:
        break
    new_bag.write(topic, msg, t=t)

ori_bag.close()
os.remove(sys.argv[1]+'.orig')
new_bag.close()
