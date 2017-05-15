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

#include "mesh.h"

mesh::mesh()
{

}

/** \brief Smooths input polygonmesh
  * \param meshToSmooth pointer to input polygonmesh
  * \param output pointer to result
  */
void mesh::smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth, pcl::PolygonMesh::Ptr output) {
    PCL_INFO("Smoothing mesh %d\n",meshToSmooth->polygons.size());
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(meshToSmooth);
    vtk.setNumIter(20000);
    vtk.setConvergence(0.1);
    vtk.setRelaxationFactor(0.1);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI);
    vtk.setBoundarySmoothing(true);
    pcl::PolygonMesh::Ptr tmp (new pcl::PolygonMesh);
    vtk.process(*tmp);
    *output = *tmp;
}


/** \brief Triangulation performed by greedy projection triangulation
  * \param cloudToPolygonate pointer to input cloud
  * \param output pointer to resultant mesh
  */
void mesh::polygonateCloudGreedyProj(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("Greedy polygonation\n");
    parameters* params = parameters::GetInstance();

    // Get Greedy result
    // normal Estimation
    pcl::NormalEstimation<PointT, pcl::Normal> normEstim;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT>);
    //pcl::search::OrganizedNeighbor<PointT>::Ptr tree2 (new pcl::search::OrganizedNeighbor<PointT>); //only for organized cloud
    tree2->setInputCloud(cloudToPolygonate);
    normEstim.setInputCloud(cloudToPolygonate);
    normEstim.setSearchMethod(tree2);
    normEstim.setKSearch(20);
    //normEstim.setNumberOfThreads(4);
    normEstim.compute(*normals);

    // concatenate the cloud with the normal fields
    PointCloudRGBNT::Ptr cloud_normals (new PointCloudRGBNT);
    pcl::concatenateFields(*cloudToPolygonate,*normals,*cloud_normals);

    pcl::search::KdTree<NormalRGBT>::Ptr tree_normal (new pcl::search::KdTree<NormalRGBT>);
    //pcl::search::OrganizedNeighbor<NormalRGBT>::Ptr tree_normal (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>); //only for organized cloud
    tree_normal->setInputCloud(cloud_normals);


    pcl::GreedyProjectionTriangulation<NormalRGBT> gp;
    // greedy proj parameters
    gp.setSearchRadius(params->GPsearchRadius);
    gp.setMu(params->GPmu);
    gp.setMaximumNearestNeighbors (params->GPmaximumNearestNeighbors);
    gp.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp.setMinimumAngle(M_PI/18); // 10 degrees
    gp.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp.setNormalConsistency(true);

    gp.setInputCloud (cloud_normals);
    gp.setSearchMethod (tree_normal);
    gp.reconstruct (*triangles);
    PCL_INFO("Polygons created: %d\n", triangles->polygons.size());
    //mesh::smoothMesh(triangles, triangles);
}


/** \brief Triangulation performed by marching cubes triangulation
  * \param cloudToPolygonate pointer to input cloud
  * \param output pointer to resultant mesh
  */
void mesh::polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("Marching cubes\n");
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree1 (new pcl::search::KdTree<PointT>);
    tree1->setInputCloud (cloudToPolygonate);
    ne.setInputCloud (cloudToPolygonate);
    ne.setSearchMethod (tree1);
    ne.setKSearch (20);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*normals);

    // concatenate the XYZ and normal fields*
    PointCloudRGBNT::Ptr cloud_with_normals (new PointCloudRGBNT);
    concatenateFields(*cloudToPolygonate, *normals, *cloud_with_normals);

    pcl::search::KdTree<NormalRGBT>::Ptr tree (new pcl::search::KdTree<NormalRGBT>);
    tree->setInputCloud (cloud_with_normals);

    PCL_INFO("begin marching cubes reconstruction\n");

    pcl::MarchingCubesHoppe<NormalRGBT> mc;
    //pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    mc.setInputCloud (cloud_with_normals);
    mc.setSearchMethod (tree);
    mc.reconstruct (*triangles);

    PCL_INFO("%d triangles created\n", triangles->polygons.size());
    //return triangles;
}

/** \brief Triangulation performed by poisson triangulation
  * \param cloudToPolygonate pointer to input cloud
  * \param output pointer to resultant mesh
  */
void mesh::polygonateCloudPoisson(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    // Get Poisson result
    parameters* params = parameters::GetInstance();

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloudToPolygonate);
    filter.filter(*filtered);

    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    //reverse normal's direction
    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    PointCloudRGBNT::Ptr cloud_smoothed_normals(new PointCloudRGBNT());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

    pcl::Poisson<NormalRGBT> poisson;
    poisson.setDepth(params->POSdepth);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(*triangles);
    PCL_INFO("mesh has %d triangles\n", triangles->polygons.size());

}

/** \brief Fills holes in resultant mesh
  * \param cloudToPolygonate pointer to input mesh
  * \param output pointer to resultant mesh
  */
void mesh::fillHoles(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut) {
    parameters* params = parameters::GetInstance();
    vtkSmartPointer<vtkPolyData> input;
    pcl::VTKUtils::mesh2vtk(*trianglesIn, input);

    vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();

    fillHolesFilter->SetInputData(input);
    fillHolesFilter->SetHoleSize(params->HOLsize);
    fillHolesFilter->Update ();

    vtkSmartPointer<vtkPolyData> polyData = fillHolesFilter->GetOutput();

    pcl::VTKUtils::vtk2mesh(polyData, *trianglesOut);
}

/** \brief Mesh decimation algorithm performed by VTK library
  * \param cloudToPolygonate pointer to input mesh
  * \param output pointer to resultant mesh
  */
void mesh::meshDecimation(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut) {
    parameters* params = parameters::GetInstance();
    pcl::MeshQuadricDecimationVTK meshDecimator;
    meshDecimator.setInputMesh(trianglesIn);
    meshDecimator.setTargetReductionFactor(params->DECtargetReductionFactor); // percents
    meshDecimator.process(*trianglesOut);
    PCL_INFO("Triangles count reduced from %d to %d\n", trianglesIn->polygons.size(), trianglesOut->polygons.size());
}

/** \brief Triangulation performed by grid projection triangulation
  * \param cloudToPolygonate pointer to input cloud
  * \param output pointer to resultant mesh
  */
void mesh::polygonateCloudGridProj(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("Grid projection polygonation\n");
    parameters* params = parameters::GetInstance();

    //Normal Estimation
    pcl::NormalEstimation<PointT, pcl::Normal> normEstim;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT>);
    normEstim.setInputCloud(cloudToPolygonate);
    normEstim.setSearchMethod(tree2);
    normEstim.setKSearch(20);
    normEstim.compute(*normals);

    // concatenate the cloud with the normal fields
    PointCloudRGBNT::Ptr cloud_normals (new PointCloudRGBNT);
    pcl::concatenateFields(*cloudToPolygonate,*normals,*cloud_normals);

    pcl::search::KdTree<NormalRGBT>::Ptr tree_normal (new pcl::search::KdTree<NormalRGBT>);
    //pcl::search::OrganizedNeighbor<NormalRGBT>::Ptr tree_normal (new pcl::search::OrganizedNeighbor<NormalRGBT>); //only for organized cloud
    tree_normal->setInputCloud(cloud_normals);

    pcl::GridProjection<NormalRGBT> gp;
    gp.setInputCloud (cloud_normals);
    gp.setSearchMethod (tree_normal);
    gp.setResolution (params->GRres);
    gp.reconstruct (*triangles);
    //retextureMesh(cloudToPolygonate, triangles);
    PCL_INFO("Polygons created: %d\n", triangles->polygons.size());
}

/** \brief Experimantal algorithm to map RGB values from input cloud to output mesh
  * \param originCloud pointer to input cloud
  * \param triangles pointer to resultant mesh
  */
void mesh::retextureMesh(PointCloudT::Ptr originCloud, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("recoloring\n");
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (originCloud);
    PointT searchPoint;
    int K = 3;

    PointCloudT::Ptr temp_cloud(new PointCloudT);
    pcl::fromPCLPointCloud2(triangles->cloud, *temp_cloud);

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i = 0; i < temp_cloud->points.size(); i++) {
        searchPoint.x = temp_cloud->points[i].x;
        searchPoint.y = temp_cloud->points[i].y;
        searchPoint.z = temp_cloud->points[i].z;

        //size_t size = pointIdxNKNSearch.size ();
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            temp_cloud->points[i].r = originCloud->points[pointIdxNKNSearch[0]].r;
            temp_cloud->points[i].g = originCloud->points[pointIdxNKNSearch[0]].g;
            temp_cloud->points[i].b = originCloud->points[pointIdxNKNSearch[0]].b;
        }
    }

    parameters* params = parameters::GetInstance();
    temp_cloud->sensor_orientation_ = params->m;
    pcl::toPCLPointCloud2(*temp_cloud,triangles->cloud);

}


