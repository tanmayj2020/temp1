#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

import pcl
import pcl_helper

def do_voxel_grid_downsampling(pcl_data, leaf_size): 
    
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size ,leaf_size ) # The bigger the leaf size the less information retained 
    return vox.filter()


def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)

    LEAF_SIZE = 0.01
    cloud = do_voxel_grid_downsampling(cloud , LEAF_SIZE)
    cloud_new = pcl_helper.pcl_to_ros(cloud)
    pub.publish(cloud_new)


if __name__ == "__main__":
    rospy.init_node("downsampling" , anonymous=True)
    rospy.Subscriber("/velodyne" , PointCloud2 , callback)

    pub = rospy.Publisher("/velodyne_points_new",PointCloud2 ,queue_size=1 )
    rospy.spin()