#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <typeinfo>

ros::Publisher pub;


void Callback(const sensor_msgs::PointCloud2 msg)
{
    std::cout<<"msg is:"<<typeid(msg).name()<<std::endl;

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg, pcl_pc);
    std::cout<<"cloud is :"<<typeid(msg).name()<<std::endl;

    sensor_msgs::PointCloud2 output;
    output = msg;
    pub.publish(output);

}


int main(int argc, char** argv){

    ros::init(argc,argv,"pcl_test_node");
    ros::NodeHandle nh;

    //create a ros subscriber
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1, Callback);
    //create a ros pub
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Output_point",1);

    ros::spin();

}
