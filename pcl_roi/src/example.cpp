#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <typeinfo>


ros::Publisher pub;

void Callback(const sensor_msgs::PointCloud2 msgs)
{


    std::cout<<"msg is "<<typeid(msgs).name()<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 output;
    // pcl::PCLPointCloud2 pcl_pc;
    // pcl::PCLPointCloud2 new_pcl;
    pcl::fromROSMsg(msgs, *cloud);

    // pcl_conversions::toPCL(msgs, cloud);



    pcl::PassThrough<pcl::PointXYZI> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,3);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5,500);
    pass.filter(*out_cloud);
    pcl::toROSMsg(*out_cloud, output);

    pub.publish(output);

}





int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_roi_example");
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    //publish
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Out_velodyne",1);
    
    ros::spin();


}


