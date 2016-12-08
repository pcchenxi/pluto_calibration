
#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// transform
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// ransic
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>​

// filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include"pcl_icp_fuser.h"

using namespace std;

ros::Publisher pub_ground, pub_kinect, pub_velodyne;

Eigen::Matrix4f transform_ir_velodyne, transform_between, transform_kinect_ir;

pcl::PointCloud<pcl::PointXYZ> velodyne_cloud, kinect_cloud;
Pcl_Icp_Fuser icp_fuser_;


tf::TransformListener* tfListener = NULL;
float sensor_height = 1.0;

void publish(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    sensor_msgs::PointCloud2 sensormsg_cloud_all;
    pcl::toROSMsg(cloud, sensormsg_cloud_all);

    pub.publish(sensormsg_cloud_all);
}

Eigen::Matrix4f get_transform(string sourse_target, string frame_target)
{
    tf::StampedTransform to_target;

    tfListener->waitForTransform(frame_target, sourse_target, ros::Time::now(), ros::Duration(1.0));
    tfListener->lookupTransform(frame_target, sourse_target, ros::Time(0), to_target);

    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (to_target, eigen_transform);
    
    return eigen_transform;
}

sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in, string frame_target)
{
////////////////////////////////// transform ////////////////////////////////////////
    sensor_msgs::PointCloud2 cloud_out;
    
    Eigen::Matrix4f eigen_transform = get_transform(cloud_in->header.frame_id, frame_target);
    
    pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_out);

    //transform_ir_velodyne = eigen_transform;
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ> cloud_filter(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr  input_cloud       (new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_voxel       (new pcl::PointCloud<pcl::PointXYZ>);

    //cout << "before filter  " << input_cloud->points.size() << endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud (input_cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-2.0, 2.0);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter (*cloud_passthrough);
    //cout << "after z filter  " << cloud_passthrough->points.size() << endl;


    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 3.0);
    pass.filter (*cloud_passthrough);
    // cout << "after x filter  " << cloud_passthrough->points.size() << endl;


    pass.setInputCloud (cloud_passthrough);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.5, 1.5);
    pass.filter (*cloud_passthrough);
    //cout << "after y filter  " << cloud_passthrough->points.size() << endl;

    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud_passthrough);
    // sor.setLeafSize (0.02f, 0.02f, 0.02f);
    // sor.filter (*cloud_voxel);
    //cout << "after voxel filter  " << cloud_voxel->points.size() << endl;

    //cout <<ros::Time::now() << "   finish filtering"<<endl;
    return *cloud_passthrough;
}

void process_cloud(pcl::PointCloud<pcl::PointXYZ> cloud_crop)
{  
    // // transform to pcl base cloud
    // pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_base_rgb, cloud_inlier;
    // copyPointCloud(pcl_cloud, cloud); 
    
    // pcl::fromROSMsg(cloud_base, pcl_cloud_base);
    // copyPointCloud(pcl_cloud_base, cloud_base_rgb); 
    // cloud_base_rgb.header.frame_id = "base_link";
   
     
    pcl::PointCloud<pcl::PointXYZ> result;
    
    ///////////////////////////////// Ransac ///////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr  input_cloud       (new pcl::PointCloud<pcl::PointXYZ>(cloud_crop));
    pcl::PointCloud<pcl::PointXYZ>  temp_inlier;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);
    
    seg.setInputCloud (input_cloud);
    seg.segment (*inliers, *coefficients);
    
    copyPointCloud(cloud_crop, *inliers, temp_inlier);
    cout << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << endl;

    temp_inlier.header.frame_id =  cloud_crop.header.frame_id;
    publish(pub_velodyne, temp_inlier);

    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

    floor_plane_normal_vector[0] = coefficients->values[0];
    floor_plane_normal_vector[1] = coefficients->values[1];
    floor_plane_normal_vector[2] = coefficients->values[2];

    //std::cout << "bloor normal " << floor_plane_normal_vector << std::endl;

    xy_plane_normal_vector[0] = 1.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 0.0;

    //std::cout << xy_plane_normal_vector << std::endl;

    rotation_vector = floor_plane_normal_vector.cross (xy_plane_normal_vector);
    std::cout << "Rotation Vector: "<< rotation_vector[0] << " " << rotation_vector[1] << " " << rotation_vector[2] << std::endl;
    
}



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fuser(pcl::PointCloud<pcl::PointXYZ>::Ptr sourse, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    // cout << "fuser" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Matrix4f pairTransform;

   // cout << "src point num: " << sourse->points.size() << "  tg num: " << target->points.size() << endl;
    icp_fuser_.pairAlign(target, sourse, output, pairTransform, true);

    pcl::transformPointCloud (*sourse, *output, pairTransform);
    //pcl::transformPointCloud (*target, *output, pairTransform);

//    Eigen::Matrix4f combi_trans = g_pairTransform*pairTransform;
//    std::cout << "Transformation Matrix: " << std::endl;
//    std::cout << pairTransform << std::endl;
    
    transform_between = pairTransform;

    return output;
}

void callback_kinect(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    ///////////// transform ////////////////////
    sensor_msgs::PointCloud2 cloud_transformed_klink = transform_cloud (cloud_in, "kinect2_link");
    cloud_transformed_klink.header.frame_id = "kinect2_link";
    
    sensor_msgs::PointCloud2 cloud_transformed = transform_cloud (cloud_in, "velodyne");
    cloud_transformed.header.frame_id = "velodyne";
        
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
   // pcl::fromROSMsg(*cloud_in, pcl_cloud);
    pcl::fromROSMsg(cloud_transformed, pcl_cloud);
    
    //pcl_cloud.header.frame_id = cloud_in->header.frame_id;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_crop = cloud_filter(pcl_cloud);
    kinect_cloud = cloud_crop;
    
    
    
    //////////////////////////////////////////////// icp fuser ////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr target       (new pcl::PointCloud<pcl::PointXYZ>(velodyne_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr source       (new pcl::PointCloud<pcl::PointXYZ>(kinect_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr output       (new pcl::PointCloud<pcl::PointXYZ>());
    
    output = cloud_fuser(source, target);
    
    transform_kinect_ir = get_transform("kinect2_link", cloud_in->header.frame_id);
    transform_ir_velodyne = get_transform(cloud_in->header.frame_id, "velodyne");
   
    Eigen::Matrix4f all_trans = transform_between * transform_ir_velodyne * transform_kinect_ir;
    
    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud (all_trans, cloud_transformed_klink, cloud_out);
    cloud_out.header.frame_id = "velodyne";
    pub_ground.publish(cloud_out);
    
    //publish(pub_ground, *output);
    publish(pub_kinect, kinect_cloud);
    publish(pub_velodyne, velodyne_cloud); 
   
   // all_trans = transform_kinect_ir * transform_between * transform_ir_velodyne;
    Eigen::Affine3f cur_trans;
    cur_trans = all_trans;

    float cur_r, cur_p, cur_y;
    pcl::getEulerAngles(cur_trans,cur_r,cur_p,cur_y);
    //std::cout << all_trans << std::endl;
    std::cout << "g roll : " << cur_r << " g pitch : " << cur_p << " g yaw : " << cur_y << " trans_x: " << all_trans(0,3) << " trans_y: " << all_trans(1,3) << " z: " << all_trans(2, 3) << std::endl;

    
    
    //process_cloud(pcl_cloud);
}

void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    ///////////// transform ////////////////////
    //sensor_msgs::PointCloud2 cloud_transformed = transform_cloud (cloud_in, "kinect_link");
    //cloud_transformed.header.frame_id = "base_link";
    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);
   // pcl::fromROSMsg(cloud_transformed, pcl_cloud);

    //pcl_cloud.header.frame_id = cloud_in->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ> cloud_crop = cloud_filter(pcl_cloud);
        
    velodyne_cloud = cloud_crop;        
        
        
   // process_cloud(cloud_crop);
   
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ransac_icp_calibration");

    ros::NodeHandle node;
    ros::Rate rate(10.0);

    ros::Subscriber sub_velodyne = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, callback_velodyne);
    ros::Subscriber sub_kinect = node.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 10, callback_kinect);

    pub_ground     = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points_ground", 1);
    pub_kinect     = node.advertise<sensor_msgs::PointCloud2>("/kinect_crop", 1);
    pub_velodyne   = node.advertise<sensor_msgs::PointCloud2>("/velodyne_crop", 1);

    // ros::Subscriber sub_odom = node.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 1, callback_odom);
    // ros::Subscriber sub_odom_icp = node.subscribe<nav_msgs::Odometry >("/icp_odom", 1, callback_odom_icp);


    tfListener = new (tf::TransformListener);

    while (node.ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
