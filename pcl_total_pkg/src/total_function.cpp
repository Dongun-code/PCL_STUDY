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


ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr point)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl_conversions::toPCL(msgs, cloud);
    pcl::PassThrough<pcl::PointXYZI> pass;
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(point);    
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10, 10);
    pass.filter(*point);
    pass.setInputCloud(point);   
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.55,0.5);
    pass.filter(*point);
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
    pcl::fromROSMsg(msgs, *cloud);

    output_cloud = point_roi(cloud);
    projection_cloud = point_projection(output_cloud);

        std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;

    //  voxel
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(projection_cloud);
    sor.setLeafSize(0.3f,0.3f,0.3f);
    sor.filter(*projection_cloud);

    // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

    //Create  object
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(projection_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(15);
    ec.setMaxClusterSize(300);
    ec.setSearchMethod(tree);
    ec.setInputCloud(projection_cloud);
    ec.extract(cluster_indices);

    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
    int j = 0;

    for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZI pt = projection_cloud->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j+1);

            TotalCloud.push_back(pt2);
            
        }
        j++;
    }

    pcl::PCLPointCloud2 cloud_clustered;
    pcl::toPCLPointCloud2(TotalCloud, cloud_clustered);
    sensor_msgs::PointCloud2 output_clustered; 
    pcl_conversions::fromPCL(cloud_clustered, output_clustered);
    output_clustered.header.frame_id = "velodyne";
    pub.publish(output_clustered); 

}





int main(int argc, char** argv)
{
    ros::init(argc,argv,"projection_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Front_velo/velodyne_points",1,Callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("Cluster_lidar_points",1);

    ros::spin();
}
