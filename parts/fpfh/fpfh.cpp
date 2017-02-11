#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>
#include <stdio.h>

int
main(int argc, char** argv)
{
    printf("run \n");
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	
	printf("nacteno \n");
	
	pcl::VoxelGrid<pcl::PointXYZ> grid;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    grid.setLeafSize (0.06, 0.06, 0.06);
    grid.setInputCloud (cloud);
    grid.filter (*cloud_src);
    

    cloud = cloud_src;
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
	
    printf("vyfiltrovano \n");
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.2);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
    
    for (int k = 0; k < normals->points.size(); k++)
    {
        if (!pcl::isFinite<pcl::Normal>(normals->points[k]))
        {
            PCL_WARN("normals[%d] is not finite\n", k);
        }
    }
    
    /*std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*normals,*normals, indices2);*/
    
    printf("normals %d\n", normals->points.size());

    int i = 0;
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud);
    printf("ok %d \n",i++);
	fpfh.setInputNormals(normals);
    printf("ok %d \n",i++);
	fpfh.setSearchMethod(kdtree);
    printf("ok %d \n",i++);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
    printf("setRadiusSearch %f\n", strtod(argv[2],NULL));
	fpfh.setRadiusSearch(strtod(argv[2],NULL));
    printf("ok %d \n",i++);
	fpfh.compute(*descriptors);
    printf("ok %d \n",i++);
}
