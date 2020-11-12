#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/ros/conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 output;

    pub.publish(output);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Pointcloud_test");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);

    ros::spin();
}