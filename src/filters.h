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
