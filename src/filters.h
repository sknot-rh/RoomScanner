#ifndef FILTERS_H
#define FILTERS_H

#include "types.h"
#include "parameters.h"
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <pcl/filters/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>


class filters
{
public:
    filters();
    static void voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float leaf = -1.0f);
    static void downsample (const PointCloudT::Ptr &input,  PointCloudT &output, double radius);
    static void cloudSmoothMLS(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
    static void cloudSmoothFBF(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
    static void oultlierRemoval(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float radius);
    static void bilatelarUpsampling(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
    static void normalFilter(PointCloudT::Ptr input, PointCloudT::Ptr output);
};

#endif // FILTERS_H
