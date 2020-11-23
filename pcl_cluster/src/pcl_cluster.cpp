// #include <iostream>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/kdtree/kdtree.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
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

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZ>::Ptr point)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl_conversions::toPCL(msgs, cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(point);    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.6,0.5);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5,500);
    pass.filter(*out_cloud);

    return out_cloud;
}

void Callback(const sensor_msgs::PointCloud2 msgs)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::fromROSMsg(msgs, *cloud);
    output_cloud = point_roi(cloud);

    std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;

    //  voxel
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f,0.5f,0.5f);
    sor.filter(*output_cloud);

    // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

    //Create  object
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(output_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(output_cloud);
    ec.extract(cluster_indices);

    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
    int j = 0;

    for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZ pt = output_cloud->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j+1);

            TotalCloud.push_back(pt2);
            
        }
        j++;
    }

    //Convert ROS MSG
    // sensor_msgs::PointCloud2 output;

    // pcl::toROSMsg(TotalCloud, output);
    // pub.publish(output);
    pcl::PCLPointCloud2 cloud_clustered;
    pcl::toPCLPointCloud2(TotalCloud, cloud_clustered);
    sensor_msgs::PointCloud2 output_clustered; 
    pcl_conversions::fromPCL(cloud_clustered, output_clustered);
    output_clustered.header.frame_id = "velodyne";
    pub.publish(output_clustered); 
}




int main(int argc,  char** argv)
{
    ros::init(argc,argv,"pcl_cluster");
    ros::NodeHandle nh;
    //subscriber
    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    //publish
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Cluster_Points",1);

    ros::spin();
}
