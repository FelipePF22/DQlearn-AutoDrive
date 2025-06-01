#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import math

class FastLidarSectorsNode:
    def __init__(self):
        rospy.init_node('fast_lidar_sectors_node')

        self.sector_pub = rospy.Publisher('/fast_lidar/sectors', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)

        rospy.loginfo("Fast Lidar Sectors Node iniciado.")
        rospy.spin()

    def lidar_callback(self, msg):
        num_sectors = 30  # 10 direita, 10 frente, 10 esquerda
        min_distances = [float('inf')] * num_sectors

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            dist = math.hypot(x, y)
            angle = math.degrees(math.atan2(y, x))

            if dist < 0.2:
                continue

            index = None
            # Direita: -90° a -30°
            if -100 <= angle < -30:
                index = int((angle + 90) / 6)
            # Frente: -30° a 30°
            elif -30 <= angle <= 30:
                index = 10 + int((angle + 30) / 6)
            # Esquerda: 30° a 90°
            elif 30 < angle <= 100:
                index = 20 + int((angle - 30) / 6)

            if index is not None and 0 <= index < num_sectors:
                if dist < min_distances[index]:
                    min_distances[index] = dist

        # Substitui inf por -1 (sem leitura)
        min_distances = [d if d != float('inf') else -1.0 for d in min_distances]

        msg_out = Float32MultiArray()
        msg_out.data = min_distances
        self.sector_pub.publish(msg_out)

if __name__ == '__main__':
    try:
        FastLidarSectorsNode()
    except rospy.ROSInterruptException:
        pass
