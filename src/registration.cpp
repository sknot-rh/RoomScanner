/*
    This file is part of RoomScanner.

    RoomScanner is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RoomScanner is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with RoomScanner.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "registration.h"

registration::registration()
{

}

PointCloudT::Ptr registration::regFrame (new PointCloudT);

/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  * \param downsample bool value if downsample input data
  */
void registration::pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample) {

    parameters *param = parameters::GetInstance();
    PointCloudT::Ptr src (new PointCloudT);
    PointCloudT::Ptr tgt (new PointCloudT);

    if (downsample)
    {
        PCL_INFO("downsampling before registration\n");
        filters::voxelGridFilter(cloud_src, src, 0.05);
        filters::voxelGridFilter(cloud_tgt, tgt, 0.05);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // compute surface normals and curvature
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

    // instantiate custom point representation
    PointRepr point_representation;
    // weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    // align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-26);
    // note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (param->REGcorrDist);
    reg.setPointRepresentation (boost::make_shared<const PointRepr> (point_representation));
    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    // Run the same optimization in a loop
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (10);

    for (int i = 0; i < 100; ++i)
    {

        PCL_INFO ("Iteration Nr. %d\n", i);
        points_with_normals_src = reg_result;
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        if (i < 98) {
            pcl::transformPointCloud (*src, *(registration::regFrame), Ti); //send undersampled output
            try {
                emit regFrameSignal();
            }
            catch (const std::length_error& le) {
            }
        }
        else {
            pcl::transformPointCloud (*cloud_src, *(registration::regFrame), Ti); //send final output
            emit regFrameSignal();
            PCL_INFO("Final output sent\n");
        }

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.9);

        prev = reg.getLastIncrementalTransformation ();
    }

    targetToSource = Ti;
    final_transform = targetToSource;

    pcl::transformPointCloud (*cloud_src, *output, targetToSource);

}

/** \brief Computes transdormation between source and target pointcloud
  * \param src_origin the source PointCloud
  * \param tgt_origin the target PointCloud
  * \return true if transformation found successfully
  */
bool registration::computeTransformation (const PointCloudT::Ptr &src_origin, const PointCloudT::Ptr &tgt_origin, Eigen::Matrix4f &transform) {
    PCL_INFO("computeTransformation\n");
    //Eigen::Matrix4f transform;

    parameters *params = parameters::GetInstance();
    PointCloudT::Ptr keypoints_src (new PointCloudT), keypoints_tgt (new PointCloudT);
    PointCloudT::Ptr src (new PointCloudT), tgt (new PointCloudT);

    filters::voxelGridFilter(src_origin, src, 0.02f); // we want downsampled copies of clouds for computation...direct downsampling would affect output quality
    filters::voxelGridFilter(tgt_origin, tgt, 0.02f);

    PCL_INFO ("after filtering clouds have %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());

    /*std::vector<int> indices; // this filtering is performed while saving frame to vector
    removeNaNFromPointCloud(*src,*src, indices);
    removeNaNFromPointCloud(*tgt,*tgt, indices);

    filters::oultlierRemoval(*src, *src, 0.5f);
    filters::oultlierRemoval(*tgt, *tgt, 0.5f);*/

    registration::estimateKeypoints (src, *keypoints_src);
    registration::estimateKeypoints (tgt, *keypoints_tgt);

    PCL_INFO ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

    if (keypoints_src->points.size() == 0 || keypoints_tgt->points.size() == 0) {
        PCL_INFO("Clouds have no key points!\n");
        return false;
    }

    // compute normals for all points keypoint
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),
        normals_tgt (new pcl::PointCloud<pcl::Normal>);
    registration::estimateNormals (src, *normals_src, params->REGnormalsRadius);
    registration::estimateNormals (tgt, *normals_tgt, params->REGnormalsRadius);
    PCL_INFO ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

    // compute FPFH features at each keypoint
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),
        fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
    registration::estimateFPFH (src, normals_src, keypoints_src, *fpfhs_src);
    registration::estimateFPFH (tgt, normals_tgt, keypoints_tgt, *fpfhs_tgt);

    // find correspondences between keypoints in FPFH space
    pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), good_correspondences (new pcl::Correspondences);
    registration::findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

    // Reject correspondences based on their XYZ distance
    registration::rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

    // obtain the best transformation between the two sets of keypoints given the remaining correspondences
    //pcl::registration::TransformationEstimationSVDScale<PointT, PointT> trans_est;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
    transformPointCloud (*src_origin, *src_origin, transform);
    return true;
}

/** \brief Rejects bad correspondences
  * \param all_correspondences all found correspondences
  * \param keypoints_src keypoints from source point cloud
  * \param keypoints_tgt keypoints from target point cloud
  * \param remaining_correspondences remaining correspondences
  */
void registration::rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
        const PointCloudT::Ptr &keypoints_src,
        const PointCloudT::Ptr &keypoints_tgt,
        pcl::Correspondences &remaining_correspondences)
{
    parameters *params = parameters::GetInstance();
    PCL_INFO("rejectBadCorrespondences\n");
    pcl::registration::CorrespondenceRejectorDistance rej;
    rej.setInputSource<PointT> (keypoints_src);
    rej.setInputTarget<PointT> (keypoints_tgt);
    rej.setMaximumDistance (params->REGreject);
    rej.setInputCorrespondences (all_correspondences);
    rej.getCorrespondences (remaining_correspondences);
}


/** \brief Finds all correspondences between fpfh features of source and target point cloud
  * \param fpfhs_src fpfh features of source point cloud
  * \param fpfhs_tgt fpfh features of target point cloud
  * \param all_correspondences target correspondences
  */
void registration::findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                                        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                                        pcl::Correspondences &all_correspondences)
{
    PCL_INFO("findCorrespondences\n");
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource (fpfhs_src);
    est.setInputTarget (fpfhs_tgt);
    est.determineReciprocalCorrespondences (all_correspondences);
}


/** \brief Finds fpfh features of point cloud
  * \param cloud input point cloud
  * \param normals estimated normals of input cloud
  * \param all_correspondences target correspondences
  * \param keypoints of input cloud
  * \param fpfh resultant fpfh
  */
void registration::estimateFPFH (const PointCloudT::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const PointCloudT::Ptr &keypoints, pcl::PointCloud<pcl::FPFHSignature33> &fpfh)
{
    PCL_INFO("estimateFPFH\n");
    parameters* params = parameters::GetInstance();
    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setNumberOfThreads(4);
    fpfh_est.setInputCloud (keypoints);
    fpfh_est.setInputNormals (normals);
    fpfh_est.setRadiusSearch (params->REGfpfh);
    fpfh_est.setSearchSurface (cloud);
    fpfh_est.compute (fpfh);
}


/** \brief Estimates normal for input point cloud
  * \param cloud input point cloud
  * \param resultant normals cloud
  * \param radius in which search for neighbors
  */
void registration::estimateNormals (const PointCloudT::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, float radius) {
   PCL_INFO("estimateNormals\n");
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
    normal_est.setNumberOfThreads(4);
    normal_est.setInputCloud (cloud);
    normal_est.setRadiusSearch (radius);
    normal_est.compute (normals);
}


/** \brief Finds keypoints of point cloud
  * \param cloud input point cloud
  * \param resultant keypoints found by SIFT algorithm
  */
void registration::estimateKeypoints (const PointCloudT::Ptr &cloud, PointCloudT &keypoints) {
    parameters* param = parameters::GetInstance();
    PCL_INFO("estimateKeypoints\n");
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    sift.setSearchMethod(tree);
    sift.setScales(param->SIFTmin_scale, param->SIFTn_octaves, param->SIFTn_scales_per_octave);
    sift.setMinimumContrast(param->SIFTmin_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);


    copyPointCloud(result, keypoints); // from PointWithScale to PointCloudAT

    PCL_INFO ("keypoints %d\n", keypoints.points.size());
    // get undersampled cloud as "3D keypoints"
    PointCloudT::Ptr depthKeypoints(new PointCloudT);
    filters::downsample(cloud, *depthKeypoints, 0.1);
    keypoints += *depthKeypoints;
    PCL_INFO ("keypoints %d\n", keypoints.points.size());
}
