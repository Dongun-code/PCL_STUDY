#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>

ros::Publisher pub;
ros::Publisher out_pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr point)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl_conversions::toPCL(msgs, cloud);
    pcl::PassThrough<pcl::PointXYZI> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud(point);    
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-10, 10);
    // pass.filter(*point);
    // pass.setInputCloud(point);   
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-1.55,0.5);
    // pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5,1000);
    pass.filter(*out_cloud);

    return out_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr point_projection(const pcl::PointCloud<pcl::PointXYZI>::Ptr point)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(point);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    return cloud_projected;    
}

void Callback(const sensor_msgs::PointCloud2 msgs)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr final(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(msgs, *cloud);

    // sensor_msgs::PointCloud2 mid_output;

    // output_cloud = point_roi(cloud);
    // projection_cloud = point_projection(output_cloud);



    std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;


    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(output_cloud);
    sor.setLeafSize(0.5f,0.5f,0.5f);
    sor.filter(*output_cloud);

    // Eigen::Vector4f min;
    // min << -30,-30,-30,1;

    // Eigen::Vector4f max;
    // max << 30,30,30,1;

    // pcl::CropBox<pcl::PointXYZI> box;
    // box.setInputCloud(projection_cloud);
    // box.setMin(min);
    // box.setMax(max);
    // box.filter(*projection_cloud);
    // std::cerr <<"ROI:"<<projection_cloud->points.size() <<std::endl;

    // Eigen::Vector4f out_min(1,1,1,1);
    // Eigen::Vector4f out_max(5,5,5,1);
    // std::vector<int> indices;

    // pcl::CropBox<pcl::PointXYZI> roof(true);
    // box.setInputCloud(projection_cloud);
    // box.setMin(out_min);
    // box.setMax(out_max);
    // box.filter(indices);
    // std::cerr <<"ROI:"<<projection_cloud->points.size() <<std::endl;

    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // for(int point:indices)
    //     inliers->indices.push_back(point);

    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setInputCloud(projection_cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*final);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (250);
    seg.setDistanceThreshold(1);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
  // While 30% of the original cloud is still there
    int i = 0, nr_points = (int) cloud->size ();
    // While 30% of the original cloud is still there

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*final);


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    // cloud_filtered.swap (cloud_f);
    i++;

    sensor_msgs::PointCloud2 in_out;
    pcl::toROSMsg(*final, in_out);
    pub.publish(in_out);

    sensor_msgs::PointCloud2 outlier_out;
    pcl::toROSMsg(*cloud_f, outlier_out);
    out_pub.publish(outlier_out);

}





int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("test_points",1);
    out_pub = nh.advertise<sensor_msgs::PointCloud2>("outlier_points",1);

    ros::spin();
}
