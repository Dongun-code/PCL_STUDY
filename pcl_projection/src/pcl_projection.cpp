#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>


ros::Publisher pub;


void Callback(const sensor_msgs::PointCloud2 msgs)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);


    pcl::fromROSMsg(msgs, *cloud);
    std::cerr << "Cloud before projection: " << std::endl;
    // for (size_t i = 0; i < cloud->points.size (); ++i)
    // std::cerr << "    " << cloud->points[i].x << " " 
    //                     << cloud->points[i].y << " " 
    //                     << cloud->points[i].z << std::endl;


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    


    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_projected,output);
    pub.publish(output);

}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"project_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("Projection_points",1);

    ros::spin();
}
