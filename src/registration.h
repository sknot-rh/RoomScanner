#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "types.h"
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include "pointrepr.h"
#include "filters.h"
#include <pcl/features/normal_3d_omp.h>


class registration
{
public:
    registration();
    static void pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
    static void estimateKeypoints (const PointCloudT::Ptr &src,
                       const PointCloudT::Ptr &tgt,
                       PointCloudT &keypoints_src,
                       PointCloudT &keypoints_tgt);
    static void estimateNormals (const PointCloudT::Ptr &src,
                     const PointCloudT::Ptr &tgt,
                     pcl::PointCloud<pcl::Normal> &normals_src,
                     pcl::PointCloud<pcl::Normal> &normals_tgt);
    static void estimateFPFH (const PointCloudT::Ptr &src,
                  const PointCloudT::Ptr &tgt,
                  const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
                  const pcl::PointCloud<pcl::Normal>::Ptr &normals_tgt,
                  const PointCloudT::Ptr &keypoints_src,
                  const PointCloudT::Ptr &keypoints_tgt,
                  pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src,
                  pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_tgt);
    static void findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                         const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                         pcl::Correspondences &all_correspondences);
    static void rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                              const PointCloudT::Ptr &keypoints_src,
                              const PointCloudT::Ptr &keypoints_tgt,
                              pcl::Correspondences &remaining_correspondences);

    static void computeTransformation (const PointCloudT::Ptr &src,
                           const PointCloudT::Ptr &tgt,
                           Eigen::Matrix4f &transform);

};

#endif // REGISTRATION_H
