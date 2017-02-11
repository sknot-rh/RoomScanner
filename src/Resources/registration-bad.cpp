PointCloudT::Ptr PCLViewer::registrateNClouds() {
    PointCloudT::Ptr result (new PointCloudT);
    PointCloudT::Ptr src, tgt;



    for (int i = 0; i < clouds.size()-1; i++) {
        tgt = clouds[i];
        src = clouds[i+1];

        // Compute the best transformtion
        Eigen::Matrix4f transform;
        PCLViewer::computeTransformation (src, tgt, transform);

        std::cerr << transform << std::endl;
        // Transform the data
        PointCloudT output;
        transformPointCloud (*src, output, transform);
        PointCloudT::Ptr tmp(&output);
        clouds[i+1] = tmp;

    }

    std::cout << "Concatenating result\n";
    for (int i = 0; i < clouds.size(); i++) {
        *result += *(clouds[i]);
    }
    viewer->removeAllPointClouds();
    viewer->addPointCloud(result, "registrated");



    std::cout << "Registrated Point Cloud has " << result->points.size() << " points.\n";
    labelRegistrate->close();
    return result;
}


void PCLViewer::computeTransformation (const PointCloudT::Ptr &src,
                       const PointCloudT::Ptr &tgt,
                       Eigen::Matrix4f &transform)
{
    std::cout << "computeTransformation\n";
  // Get an uniform grid of keypoints
  PointCloudT::Ptr keypoints_src (new PointCloudT), keypoints_tgt (new PointCloudT);

  PCLViewer::estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
  printf ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

  // Compute normals for all points keypoint
  pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),
                          normals_tgt (new pcl::PointCloud<pcl::Normal>);
  PCLViewer::estimateNormals (src, tgt, *normals_src, *normals_tgt);
  printf ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

  // Compute FPFH features at each keypoint
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),
                                   fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
  PCLViewer::estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

  // Find correspondences between keypoints in FPFH space
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences),
                     good_correspondences (new pcl::Correspondences);
  PCLViewer::findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

  // Reject correspondences based on their XYZ distance
  PCLViewer::rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

  for (int i = 0; i < good_correspondences->size (); ++i)
    std::cerr << good_correspondences->at (i) << std::endl;
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
}


void PCLViewer::rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
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

void PCLViewer::findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                     pcl::Correspondences &all_correspondences)
{
    std::cout << "findCorrespondences\n";
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}

void PCLViewer::estimateFPFH (const PointCloudT::Ptr &src,
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

void PCLViewer::estimateNormals (const PointCloudT::Ptr &src,
                 const PointCloudT::Ptr &tgt,
                 pcl::PointCloud<pcl::Normal> &normals_src,
                 pcl::PointCloud<pcl::Normal> &normals_tgt)
{
    std::cout << "estimateNormals\n";
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
  normal_est.setNumberOfThreads(4);
  normal_est.setInputCloud (src);
  std::cout << "normals source\n";
  normal_est.setRadiusSearch (0.5);  // 50cm
  normal_est.compute (normals_src);

  std::cout << "normals target\n";
  normal_est.setInputCloud (tgt);
  normal_est.compute (normals_tgt);
}

void PCLViewer::estimateKeypoints (const PointCloudT::Ptr &src,
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
