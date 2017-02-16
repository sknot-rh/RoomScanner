#include "registration.h"

registration::registration()
{

}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void registration::pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloudT::Ptr src (new PointCloudT);
  PointCloudT::Ptr tgt (new PointCloudT);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
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
  PointRepr point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-16);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.75);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const PointRepr> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (20);
  for (int i = 0; i < 100; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
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
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.9);

    prev = reg.getLastIncrementalTransformation ();

  }

    //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }


void registration::computeTransformation (const PointCloudT::Ptr &src_origin,
                       const PointCloudT::Ptr &tgt_origin,
                       Eigen::Matrix4f &transform)
{
    std::cout << "computeTransformation\n";
  // Get an uniform grid of keypoints
  PointCloudT::Ptr keypoints_src (new PointCloudT), keypoints_tgt (new PointCloudT);

  PointCloudT::Ptr src (new PointCloudT),
                   tgt (new PointCloudT);

  filters::downsample(src_origin, *src);
  filters::downsample(tgt_origin, *tgt);

  registration::estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
  printf ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

  // Compute normals for all points keypoint
  pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),
                          normals_tgt (new pcl::PointCloud<pcl::Normal>);
  registration::estimateNormals (src, tgt, *normals_src, *normals_tgt);
  printf ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

  // Compute FPFH features at each keypoint
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),
                                   fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
  registration::estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

  // Find correspondences between keypoints in FPFH space
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences),
                     good_correspondences (new pcl::Correspondences);
  registration::findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

  // Reject correspondences based on their XYZ distance
  registration::rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

  for (int i = 0; i < good_correspondences->size (); ++i)
    std::cerr << good_correspondences->at (i) << std::endl;
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
}


void registration::rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const PointCloudT::Ptr &keypoints_src,
                          const PointCloudT::Ptr &keypoints_tgt,
                          pcl::Correspondences &remaining_correspondences)
{
    std::cout << "rejectBadCorrespondences\n";
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputCloud<PointT> (keypoints_src);
  rej.setInputTarget<PointT> (keypoints_tgt);
  rej.setMaximumDistance (1);    // 1m
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}

void registration::findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                     pcl::Correspondences &all_correspondences)
{
    std::cout << "findCorrespondences\n";
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}

void registration::estimateFPFH (const PointCloudT::Ptr &src,
              const PointCloudT::Ptr &tgt,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_tgt,
              const PointCloudT::Ptr &keypoints_src,
              const PointCloudT::Ptr &keypoints_tgt,
              pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src,
              pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_tgt)
{
    std::cout << "estimateFPFH\n";
  pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (keypoints_src);
  fpfh_est.setInputNormals (normals_src);
  fpfh_est.setRadiusSearch (1); // 1m
  fpfh_est.setSearchSurface (src);
  fpfh_est.compute (fpfhs_src);

  fpfh_est.setInputCloud (keypoints_tgt);
  fpfh_est.setInputNormals (normals_tgt);
  fpfh_est.setSearchSurface (tgt);
  fpfh_est.compute (fpfhs_tgt);
}

void registration::estimateNormals (const PointCloudT::Ptr &src,
                 const PointCloudT::Ptr &tgt,
                 pcl::PointCloud<pcl::Normal> &normals_src,
                 pcl::PointCloud<pcl::Normal> &normals_tgt)
{
    std::cout << "estimateNormals\n";
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
  normal_est.setNumberOfThreads(4);
  normal_est.setInputCloud (src);
  std::cout << "normals source " << src->points.size() << "\n";
  normal_est.setRadiusSearch (0.5);  // 50cm
  normal_est.compute (normals_src);

  std::cout << "normals target " << tgt->points.size() << "\n";
  normal_est.setInputCloud (tgt);
  normal_est.compute (normals_tgt);
}

void registration::estimateKeypoints (const PointCloudT::Ptr &src,
                   const PointCloudT::Ptr &tgt,
                   PointCloudT &keypoints_src,
                   PointCloudT &keypoints_tgt)
{
    std::cout << "estimateKeypoints\n";
  // Get an uniform grid of keypoints
  pcl::UniformSampling<PointT> uniform;
  uniform.setRadiusSearch (1);  // 1m

  uniform.setInputCloud (src);
  uniform.filter (keypoints_src);

  uniform.setInputCloud (tgt);
  uniform.filter (keypoints_tgt);
}
