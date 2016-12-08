#ifndef PCL_ICP_FUSER_H_
#define PCL_ICP_FUSER_H_

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h> 
#include <pcl/registration/transforms.h>

///////////////////////////////////////////////////////
#include <pcl/visualization/pcl_visualizer.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;



using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class Pcl_Icp_Fuser
{
    public:
    Pcl_Icp_Fuser();
    ~Pcl_Icp_Fuser();

    pcl::visualization::PCLVisualizer *p;
    //its left and right viewports

    int vp_1, vp_2;

    void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
    void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);
    void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
    public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

Pcl_Icp_Fuser::Pcl_Icp_Fuser()
{

//	p = new pcl::visualization::PCLVisualizer ("Pairwise Incremental Registration example");
//  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
}

Pcl_Icp_Fuser::~Pcl_Icp_Fuser()
{
}

void Pcl_Icp_Fuser::showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}

void Pcl_Icp_Fuser::showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}


void Pcl_Icp_Fuser::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  // Downsample for consistency and speed
  // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;

    if (downsample)
    {
        grid.setLeafSize (0.03, 0.03, 0.03);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);
        //src = cloud_src;
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

   // cout << "src point num: " << cloud_src->points.size() << "  tg num: " << cloud_tgt->points.size() << endl;
   // cout << "src point num: " << src->points.size() << "  tg num: " << tgt->points.size() << endl;
    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-10);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets

    float init_dist = 0.1;
    reg.setMaxCorrespondenceDistance (init_dist);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (100);
    for (int i = 0; i < 11; ++i)
    {
        //PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose

    reg.setMaxCorrespondenceDistance (init_dist);
      //  init_dist -= 0.005;
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

            //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();
//        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
//          p->spin ();
    }

    // Get the transformation from target to source
    targetToSource = Ti.inverse();
    //targetToSource = Ti;




//
//    // Transform target back in source frame
//    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
//      p->removePointCloud ("source");
//  p->removePointCloud ("target");
//
//  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
//  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
//  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
//  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
//
//	PCL_INFO ("Press q to continue the registration.\n");
//  p->spin ();
//
//  p->removePointCloud ("source");
//  p->removePointCloud ("target");


    //*output += *cloud_src;

    final_transform = targetToSource;
}

#endif
