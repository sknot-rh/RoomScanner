#include "mesh.h"

mesh::mesh()
{

}

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

void mesh::polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("Greedy polygonation\n");
    parameters* params = parameters::GetInstance();

    // Get Greedy result
    //Normal Estimation
    pcl::NormalEstimation<PointT, pcl::Normal> normEstim;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT>);
    //pcl::search::OrganizedNeighbor<PointT>::Ptr tree2 (new pcl::search::OrganizedNeighbor<PointT>); //only for organized cloud
    tree2->setInputCloud(cloudToPolygonate);//cloud_filtered
    normEstim.setInputCloud(cloudToPolygonate);//cloud_filtered
    normEstim.setSearchMethod(tree2);
    normEstim.setKSearch(20);
    //normEstim.setNumberOfThreads(4);
    normEstim.compute(*normals);

    //Concatenate the cloud with the normal fields
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloudToPolygonate,*normals,*cloud_normals);

    //Create  search tree to include cloud with normals
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    //pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>::Ptr tree_normal (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>); //only for organized cloud
    tree_normal->setInputCloud(cloud_normals);


    //Initialize objects for triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp;
    //boost::shared_ptr<pcl::PolygonMesh> triangles(new pcl::PolygonMesh);
    //pcl::PolygonMesh triangles;

    //Max distance between connecting edge points
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
    mesh::smoothMesh(triangles, triangles);
}

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

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    concatenateFields(*cloudToPolygonate, *normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud (cloud_with_normals);

    PCL_INFO("begin marching cubes reconstruction\n");

    pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> mc;
    //pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    mc.setInputCloud (cloud_with_normals);
    mc.setSearchMethod (tree);
    mc.reconstruct (*triangles);

    PCL_INFO("%d triangles created\n", triangles->polygons.size());
    //return triangles;
}


void mesh::polygonateCloudPoisson(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {

    // Get Poisson result
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

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(*triangles);
    PCL_INFO("mesh has %d triangles\n", triangles->polygons.size());

}


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

void mesh::meshDecimation(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut) {
    parameters* params = parameters::GetInstance();
    pcl::MeshQuadricDecimationVTK meshDecimator;
    meshDecimator.setInputMesh(trianglesIn);
    meshDecimator.setTargetReductionFactor(params->DECtargetReductionFactor); // percents
    meshDecimator.process(*trianglesOut);
    PCL_INFO("Triangles count reduced from %d to %d\n", trianglesIn->polygons.size(), trianglesOut->polygons.size());
}


void mesh::polygonateCloudGridProj(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("Grid projection polygonation\n");
    parameters* params = parameters::GetInstance();


    // Get Greedy result
    //Normal Estimation
    pcl::NormalEstimation<PointT, pcl::Normal> normEstim;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT>);
    //pcl::search::OrganizedNeighbor<PointT>::Ptr tree2 (new pcl::search::OrganizedNeighbor<PointT>); //only for organized cloud
    tree2->setInputCloud(cloudToPolygonate);//cloud_filtered
    normEstim.setInputCloud(cloudToPolygonate);//cloud_filtered
    normEstim.setSearchMethod(tree2);
    normEstim.setKSearch(20);
    //normEstim.setNumberOfThreads(4);
    normEstim.compute(*normals);

    //Concatenate the cloud with the normal fields
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloudToPolygonate,*normals,*cloud_normals);

    //Create  search tree to include cloud with normals
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    //pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>::Ptr tree_normal (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>); //only for organized cloud
    tree_normal->setInputCloud(cloud_normals);


    //Initialize objects for triangulation
    pcl::GridProjection<pcl::PointXYZRGBNormal> gp;
    //boost::shared_ptr<pcl::PolygonMesh> triangles(new pcl::PolygonMesh);
    //pcl::PolygonMesh triangles;

    //Max distance between connecting edge points
    //gp.setSearchRadius(params->GPsearchRadius);

    gp.setInputCloud (cloud_normals);
    gp.setSearchMethod (tree_normal);
    gp.setResolution (params->GRres);
    gp.reconstruct (*triangles);
    //retextureMesh(cloudToPolygonate, triangles);
    PCL_INFO("Polygons created: %f\n", triangles->polygons.size());
}

void mesh::retextureMesh(PointCloudT::Ptr originCloud, pcl::PolygonMesh::Ptr triangles) {
    PCL_INFO("recoloring\n");
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (originCloud);
    pcl::PointXYZRGB searchPoint;
    int K = 3;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(triangles->cloud, *temp_cloud);

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i = 0; i < temp_cloud->points.size(); i++) {
        searchPoint.x = temp_cloud->points[i].x;
        searchPoint.y = temp_cloud->points[i].y;
        searchPoint.z = temp_cloud->points[i].z;

        size_t size = pointIdxNKNSearch.size ();
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


