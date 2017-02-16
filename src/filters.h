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



class filters
{
public:
    filters();
    static void voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered);
    static void downsample (const PointCloudT::Ptr &src_origin,  PointCloudT &src);
    static void cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
    static void oultlierRemoval(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered);
};

#endif // FILTERS_H
