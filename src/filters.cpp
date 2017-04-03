#include "filters.h"

filters::filters()
{

}

/** \brief Voxel grid filter
  * \param cloudToFilter pointer to input cloud
  * \param filtered pointer to resultant cloud
  * \param leaf size of cubic voxel
  */
void filters::voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float leaf) {
    PCL_INFO("downsampling filter\n");

    parameters* params = parameters::GetInstance();
    pcl::VoxelGrid<PointT> ds;  //create downsampling filter
    ds.setInputCloud (cloudToFilter);
    if (leaf > 0.00000f) {
        PCL_INFO("VoxelGrid parameter with priority %f\n", leaf);
        ds.setLeafSize (leaf, leaf, leaf);
    }
    else {
        ds.setLeafSize (params->VGFleafSize, params->VGFleafSize, params->VGFleafSize);
    }
    ds.filter (*filtered);
    PCL_INFO("Filtered points: %d\n", filtered->points.size());
}

/** \brief Downsampling performed by uniform sampling filter
  * \param cloudToFilter pointer to input cloud
  * \param filtered pointer to resultant cloud
  * \param radius size of neighborhood
  */
void filters::downsample (const PointCloudT::Ptr &input,  PointCloudT &output, double radius) {
    // Get an uniform grid of keypoints
    pcl::UniformSampling<PointT> uniform;
    uniform.setRadiusSearch (radius);

    uniform.setInputCloud (input);
    uniform.filter (output);
}

/** \brief Smoothing of input cloud performed by MLS
  * \param cloudToSmooth pointer to input cloud
  * \param output pointer to resultant cloud
  */
void filters::cloudSmoothMLS(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output) {
    PCL_INFO("smoothing %d points\n", cloudToSmooth->points.size());

    parameters* params = parameters::GetInstance();

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud (cloudToSmooth);
    mls.setSearchMethod (tree);
    mls.setComputeNormals (params->MLScomputeNormals);
    mls.setSearchRadius (params->MLSsearchRadius);
    mls.setSqrGaussParam (params->MLSsqrGaussParam);
    mls.setPolynomialFit (params->MLSusePolynomialFit);
    mls.setPolynomialOrder (params->MLSpolynomialOrder);
    mls.setUpsamplingRadius (params->MLSupsamplingRadius);
    mls.setUpsamplingStepSize (params->MLSupsamplingStepSize);
    mls.setDilationVoxelSize (params->MLSdilationVoxelSize);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);

    // Available upsampling methods
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointXYZRGB>::NONE);

    PointCloudT::Ptr cloud_smoothed (new PointCloudT ());
    mls.process (*cloud_smoothed);
    //output = cloud_smoothed;
    copyPointCloud(*cloud_smoothed, *output);
    PCL_INFO("Smoothed cloud has %d points\n", output->points.size());
}

/** \brief Outlier removal
  * \param cloudToFilter pointer to input cloud
  * \param filtered pointer to resultant cloud
  * \param radius to search close points
  */
void filters::oultlierRemoval(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float radius) {
    /*pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloudToFilter);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered);*/

    pcl::RadiusOutlierRemoval<PointT> rorfilter;
    rorfilter.setInputCloud (cloudToFilter);
    rorfilter.setRadiusSearch (radius);
    rorfilter.setMinNeighborsInRadius (5);
    rorfilter.filter (*filtered);
}

/** \brief Smooth organized point cloud with Fast Bilateral Filter
  * \param cloudToSmooth pointer to input cloud
  * \param output pointer to resultant cloud
  */
void filters::cloudSmoothFBF(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output) {
    PCL_INFO("FBFilter\n");
    parameters* params = parameters::GetInstance();
    pcl::FastBilateralFilter<PointT> filter;
    filter.setInputCloud(cloudToSmooth);
    filter.setSigmaS(params->FBFsigmaS);
    filter.setSigmaR(params->FBFsigmaR);
    filter.applyFilter(*output);
}

/** \brief Experimental bilateral upsampling
  * \param cloudToSmooth pointer to input cloud
  * \param output pointer to resultant cloud
  */
void filters::bilatelarUpsampling(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output) {
    pcl::BilateralUpsampling<PointT, PointT> bu;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloudToSmooth, *tmp);
    bu.setInputCloud (cloudToSmooth);

    int window_size = 15;
    double sigma_color = 15;
    double sigma_depth = 15;

    bu.setWindowSize (window_size);
    bu.setSigmaColor (sigma_color);
    bu.setSigmaDepth (sigma_depth);


    bu.setProjectionMatrix (bu.KinectSXGAProjectionMatrix);
    bu.process (*output);

    for (int i = 0; i < cloudToSmooth->points.size(); i++) {
        //output->points[i].rgb = original->points[i].rgb;
          tmp->points[i].x = output->points[i].x;
          tmp->points[i].y = output->points[i].y;
          tmp->points[i].z = output->points[i].z;

      }

    pcl::copyPointCloud(*tmp, *output);
    PCL_INFO("output has %d points\n", output->points.size());
}

/** \brief Perform sampling in normal space
  * \param input pointer to input cloud
  * \param output pointer to resultant cloud
  */
void filters::normalFilter(PointCloudT::Ptr input, PointCloudT::Ptr output) {
    PCL_INFO("normalFilter\n");
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (input);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.02);
    ne.compute (*cloud_normals);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*input, *cloud_normals, *temp);

    pcl::NormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> normal_space_sampling;
    normal_space_sampling.setInputCloud (temp);
    normal_space_sampling.setNormals (temp);
    normal_space_sampling.setBins (16,16,16);
    normal_space_sampling.setSeed (42);
    normal_space_sampling.setSample (static_cast<unsigned int> (input->size ()-1));
    normal_space_sampling.filter(*temp);
    pcl::copyPointCloud(*temp, *output);
    //filters::voxelGridFilter(output, output, 0.02);
}

