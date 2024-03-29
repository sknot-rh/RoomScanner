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

#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointAT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointAT> PointCloudAT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointXYZRGBNormal NormalRGBT;
typedef pcl::PointCloud<NormalRGBT> PointCloudRGBNT;

#endif // TYPES_H
