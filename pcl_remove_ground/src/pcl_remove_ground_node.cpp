#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


ros::Publisher pub;

void Callback(const sensor_msgs::PointCloud2ConstPtr& point)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*point, cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    std::cout<<*coefficients<<std::endl;
    std::cout<<*inliers<<std::endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"Remove_ground_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    // pub = nh.advertise<sensor_msgs::PointCloud2>("/remove_ground",1);
    // #pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub = nh.advertise<pcl_msgs::ModelCoefficients>("/remove_ground",1);

    ros::spin();

}
