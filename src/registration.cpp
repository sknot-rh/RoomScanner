#include "registration.h"

registration::registration()
{

}

PointCloudT::Ptr registration::regFrame (new PointCloudT);

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void registration::pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{

    parameters *param = parameters::GetInstance();
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
    reg.setTransformationEpsilon (1e-26);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (param->REGcorrDist);
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

        if (i < 98) {
            pcl::transformPointCloud (*src, *(registration::regFrame), Ti); //send undersampled output
            emit regFrameSignal();
        }
        else {
            pcl::transformPointCloud (*cloud_src, *(registration::regFrame), Ti); //send final output
            emit regFrameSignal();
            printf("Final output sent\n");

        }


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


bool registration::computeTransformation (const PointCloudT::Ptr &src_origin, const PointCloudT::Ptr &tgt_origin)
{
    std::cout << "computeTransformation\n";
    Eigen::Matrix4f transform;

    parameters *params = parameters::GetInstance();
    PointCloudT::Ptr keypoints_src (new PointCloudT), keypoints_tgt (new PointCloudT);
    PointCloudT::Ptr src (new PointCloudT), tgt (new PointCloudT);

    //PointCloudT::Ptr src (new PointCloudT), tgt (new PointCloudT);


    // Preprocessing
    filters::voxelGridFilter(src_origin, src, 0.05f); // we want downsampled copies of clouds for computation...direct downsampling would affect output quality
    filters::voxelGridFilter(tgt_origin, tgt, 0.05f);

    printf ("after filtering clouds have %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());

    /*std::vector<int> indices; // this filtering is performed while saving frame to vector
    removeNaNFromPointCloud(*src,*src, indices);
    removeNaNFromPointCloud(*tgt,*tgt, indices);

    filters::oultlierRemoval(*src, *src, 0.5f);
    filters::oultlierRemoval(*tgt, *tgt, 0.5f);*/

    registration::estimateKeypoints (src, *keypoints_src);
    registration::estimateKeypoints (tgt, *keypoints_tgt);

    printf ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

    if (keypoints_src->points.size() == 0 || keypoints_tgt->points.size() == 0) {
        printf("Clouds have no key points!\n");
        return false;
    }

    // Compute normals for all points keypoint
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),
        normals_tgt (new pcl::PointCloud<pcl::Normal>);
    registration::estimateNormals (src, *normals_src, params->REGnormalsRadius);
    registration::estimateNormals (tgt, *normals_tgt, params->REGnormalsRadius);
    printf ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

    // Compute FPFH features at each keypoint
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),
        fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
    registration::estimateFPFH (src, normals_src, keypoints_src, *fpfhs_src);
    registration::estimateFPFH (tgt, normals_tgt, keypoints_tgt, *fpfhs_tgt);

    // Find correspondences between keypoints in FPFH space
    pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), good_correspondences (new pcl::Correspondences);
    registration::findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

    // Reject correspondences based on their XYZ distance
    registration::rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

    // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
    //pcl::registration::TransformationEstimationSVDScale<PointT, PointT> trans_est;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
    transformPointCloud (*src_origin, *src_origin, transform);
}


void registration::rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
        const PointCloudT::Ptr &keypoints_src,
        const PointCloudT::Ptr &keypoints_tgt,
        pcl::Correspondences &remaining_correspondences)
{
    parameters *params = parameters::GetInstance();
    std::cout << "rejectBadCorrespondences\n";
    pcl::registration::CorrespondenceRejectorDistance rej;
    rej.setInputSource<PointT> (keypoints_src);
    rej.setInputTarget<PointT> (keypoints_tgt);
    rej.setMaximumDistance (params->REGreject);
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

void registration::estimateFPFH (const PointCloudT::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const PointCloudT::Ptr &keypoints, pcl::PointCloud<pcl::FPFHSignature33> &fpfhs)
{
    std::cout << "estimateFPFH\n";
    parameters* params = parameters::GetInstance();
    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setNumberOfThreads(4);
    fpfh_est.setInputCloud (keypoints);
    fpfh_est.setInputNormals (normals);
    fpfh_est.setRadiusSearch (params->REGfpfh);
    fpfh_est.setSearchSurface (cloud);
    fpfh_est.compute (fpfhs);
}

void registration::estimateNormals (const PointCloudT::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, float radius)
{
    std::cout << "estimateNormals\n";
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
    normal_est.setNumberOfThreads(4);
    normal_est.setInputCloud (cloud);
    normal_est.setRadiusSearch (radius);
    normal_est.compute (normals);
}

void registration::estimateKeypoints (const PointCloudT::Ptr &cloud, PointCloudT &keypoints)
{
    parameters* param = parameters::GetInstance();
    std::cout << "estimateKeypoints\n";
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    sift.setSearchMethod(tree);
    sift.setScales(param->SIFTmin_scale, param->SIFTn_octaves, param->SIFTn_scales_per_octave);
    sift.setMinimumContrast(param->SIFTmin_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);


    copyPointCloud(result, keypoints); // from PointWithScale to PointCloudAT

    printf ("keypoints %d\n", keypoints.points.size());
    // get undersampled cloud as "3D keypoints"
    PointCloudT::Ptr depthKeypoints(new PointCloudT);
    filters::downsample(cloud, *depthKeypoints, 0.1);
    keypoints += *depthKeypoints;
    printf ("keypoints %d\n", keypoints.points.size());
}
