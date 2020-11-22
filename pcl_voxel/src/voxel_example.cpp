#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZI> point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr point)
{

    pcl::PointCloud<pcl::PointXYZI> out_cloud;


    // pcl_conversions::toPCL(msgs, cloud);



    pcl::PassThrough<pcl::PointXYZI> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(point);    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.3,1);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5,500);
    pass.filter(out_cloud);

    return out_cloud;

}

void Callback(const sensor_msgs::PointCloud2 msgs)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> output_cloud;


    pcl::fromROSMsg(msgs, *cloud);

    std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.3f,0.3f,0.3f);
    sor.filter(*cloud_filtered);

    output_cloud = point_roi(cloud_filtered);

    std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(output_cloud, output);
    pub.publish(output);
    

}




int main(int argc, char** argv)
{

    ros::init(argc,argv,"pcl_voxel_example");
    ros::NodeHandle nh;
    //subscriber
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    //publish
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Voxel_points",1);

    ros::spin();

}