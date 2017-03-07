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
#include "parameters.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <QObject>
#include <pcl/registration/transformation_estimation_svd_scale.h>

class registration : public QObject
{
    Q_OBJECT

public:
    registration();
    void pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
    static void estimateKeypoints (const PointCloudT::Ptr &cloud, PointCloudT &keypoints);
    static void estimateNormals (const PointCloudT::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, float radius);
    static void estimateFPFH (const PointCloudT::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const PointCloudT::Ptr &keypoints, pcl::PointCloud<pcl::FPFHSignature33> &fpfhs);
    static void findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                                     pcl::Correspondences &all_correspondences);
    static void rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                                          const PointCloudT::Ptr &keypoints_src,
                                          const PointCloudT::Ptr &keypoints_tgt,
                                          pcl::Correspondences &remaining_correspondences);
    bool computeTransformation (const PointCloudT::Ptr &src, const PointCloudT::Ptr &tgt);

    static PointCloudT::Ptr regFrame;

signals:
    void regFrameSignal(void);
};

#endif // REGISTRATION_H
