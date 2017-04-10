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
