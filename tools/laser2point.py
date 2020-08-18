#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

pub_ptcl = None

def laserCb(msg):
    projector = LaserProjection()
    cloud_out = projector.projectLaser(msg)
    pub_ptcl.publish(cloud_out)

    


if __name__ == '__main__':
    rospy.init_node('lidar2pointcloud')
    rospy.Subscriber('/lidar/laserscan',LaserScan,laserCb)
    pub_ptcl = rospy.Publisher("/lidar/ptcl", PointCloud2, queue_size=50)
    rospy.spin()
