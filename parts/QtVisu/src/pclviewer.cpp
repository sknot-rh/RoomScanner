#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <cstddef>
#include <iterator>
#include <list>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <QTimer>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <QFileDialog>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/search/organized.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes.h>
#include <fstream>
#include <cstdio>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"



PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer) {
    ui->setupUi (this);
    this->setWindowTitle ("RoomScanner");

    // Timer for cloud & UI update
    QTimer *tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(drawFrame()));

    //Create empty clouds
    cloud.reset(new PointCloudAT);
    key_cloud.reset(new PointCloudAT);


    //Tell to sensor in which position is expected input
    Eigen::Quaternionf m;
    m = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(0.0f,  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ());
    cloud->sensor_orientation_ = m;
    key_cloud->sensor_orientation_ = m;

    copying = stream = false;
    sensorConnected = false;

    try {
        //OpenNIGrabber
        interface = new pcl::OpenNIGrabber();
        sensorConnected = true;
    }
    catch (pcl::IOException e) {
        std::cout << "No sensor connected!" << '\n';
        sensorConnected = false;
    }

    //Setting up UI
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    meshViewer.reset (new pcl::visualization::PCLVisualizer ("meshViewer", false));

    ui->qvtkWidget_2->SetRenderWindow (meshViewer->getRenderWindow ());
    meshViewer->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
    ui->qvtkWidget_2->update ();

    //Create callback for openni grabber
    if (sensorConnected) {
        boost::function<void (const PointCloudAT::ConstPtr&)> f = boost::bind (&PCLViewer::cloud_cb_, this, _1);
        interface->registerCallback(f);
        interface->start ();
        tmrTimer->start(20); // msec
    }

    stream = true;
    stop = false;


    //Connect reset button
    connect(ui->pushButton_reset, SIGNAL (clicked ()), this, SLOT (resetButtonPressed ()));

    //Connect save button
    connect(ui->pushButton_save, SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));

    //Connect save button
    connect(ui->pushButton_poly, SIGNAL (clicked ()), this, SLOT (polyButtonPressed ()));

    // Connect point size slider
    connect(ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    //Connect checkbox
    connect(ui->checkBox, SIGNAL(clicked(bool)), this, SLOT(toggled(bool)));

    //Connect menu checkboc - show last frame
    connect(ui->actionShow_captured_frames, SIGNAL(triggered()), this, SLOT(lastFrameToggled()));

    //Connect load action
    connect(ui->actionLoad_Point_Cloud, SIGNAL (triggered()), this, SLOT (loadActionPressed ()));

    //Add empty pointclouds
    viewer->addPointCloud(cloud, "cloud");

    viewer->addPointCloud(key_cloud, "keypoints");


    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
    pSliderValueChanged (2);
    ui->tabWidget->setCurrentIndex(0);

}


void PCLViewer::drawFrame() {
    if (!stop) {
        if (mtx_.try_lock()) {
            cloud->clear();
            cloud->width = cloudWidth;
            cloud->height = cloudHeight;
            cloud->points.resize(cloudHeight*cloudWidth);
            cloud->is_dense = false;
            // Fill cloud
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];
            for(int i = 0; i < cloud->points.size();i++,pX++,pY++,pZ++,pRGB++) {
                cloud->points[i].x = (*pX);
                cloud->points[i].y = (*pY);
                cloud->points[i].z = (*pZ);
                cloud->points[i].rgba = (*pRGB);
                //cloud->points[i].a = 128; //for better stitching?
            }
            mtx_.unlock();
        }


        if (ui->actionShow_keypoints->isChecked() == true) {
            // Estimate the sift interest points using Intensity values from RGB values
            pcl::SIFTKeypoint<PointAT, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<PointAT>::Ptr tree(new pcl::search::KdTree<PointAT> ());
            sift.setSearchMethod(tree);
            sift.setScales(min_scale, n_octaves, n_scales_per_octave);
            sift.setMinimumContrast(min_contrast);
            sift.setInputCloud(cloud);
            sift.compute(result);

            copyPointCloud(result, *key_cloud);

            for (int var = 0; var < key_cloud->size(); ++var) {
                key_cloud->points[var].r = 0;
                key_cloud->points[var].g = 255;
                key_cloud->points[var].b = 0;

            }

            viewer->updatePointCloud(key_cloud ,"keypoints");

        }
        viewer->updatePointCloud(cloud ,"cloud");
        ui->qvtkWidget->update ();
    }

}

void PCLViewer::cloud_cb_ (const PointCloudAT::ConstPtr &ncloud) {

    if (stream) {
        if (mtx_.try_lock()) {

            // Size of cloud
            cloudWidth = ncloud->width;
            cloudHeight = ncloud->height;

            // Resize the XYZ and RGB point vector
            size_t newSize = ncloud->height*ncloud->width;
            cloudX.resize(newSize);
            cloudY.resize(newSize);
            cloudZ.resize(newSize);
            cloudRGB.resize(newSize);

            // Assign pointers to copy data
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];

            // Copy data (using pcl::copyPointCloud, the color stream jitters!!! Why?)
            //pcl::copyPointCloud(*ncloud, *cloud);
            for (int j = 0;j<ncloud->height;j++){
                for (int i = 0;i<ncloud->width;i++,pX++,pY++,pZ++,pRGB++) {
                    PointAT P = ncloud->at(i,j);
                    (*pX) = P.x;
                    (*pY) = P.y;
                    (*pZ) = P.z;
                    (*pRGB) = P.rgba;
                }
            }
            // Data copied
            mtx_.unlock();
        }
    }
}

void PCLViewer::toggled(bool value) {
    if (value) {
        viewer->addCoordinateSystem(1, 0, 0, 0, "viewer", 0);
    }
    else {
        viewer->removeCoordinateSystem("viewer", 0);
    }
    ui->qvtkWidget->update();
}

void PCLViewer::resetButtonPressed() {
    viewer->resetCamera();
    ui->qvtkWidget->update();
}

void PCLViewer::saveButtonPressed() {
    clouds.push_back(cloud);
    lastFrameToggled();
}

void PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->lcdNumber_p->display(value);
  ui->qvtkWidget->update ();
}

void PCLViewer::loadActionPressed() {
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Choose Point Cloud"), "/home", tr("Point Cloud Files (*.pcd)"));
    std::string utf8_fileName = fileName.toUtf8().constData();
    PointCloudAT::Ptr cloud2 (new PointCloudAT);
    if (pcl::io::loadPCDFile<PointAT> (utf8_fileName, *cloud2) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file!\n");
        return;
    }
    for (size_t i = 0; i < cloud2->size(); i++)
    {
        cloud2->points[i].a = 255;
    }

    ui->tabWidget->setCurrentIndex(0);
    std::cout << "PC Loaded ";
    viewer->removePointCloud("cloud2");
    viewer->addPointCloud(cloud2, "cloud2");
    viewer->updatePointCloud(cloud2 ,"cloud2");
    ui->qvtkWidget->update();
}

void PCLViewer::polyButtonPressed() {
    PointCloudT::Ptr cloudtmp (new PointCloudT);
    PointCloudT::Ptr output (new PointCloudT);
    cloudtmp->clear();
    //keep point cloud organized
    cloudtmp->width = cloudWidth;
    cloudtmp->height = cloudHeight;
    cloudtmp->points.resize(cloudHeight*cloudWidth);
    cloudtmp->is_dense = false;
    // Fill cloud
    float *pX = &cloudX[0];
    float *pY = &cloudY[0];
    float *pZ = &cloudZ[0];
    unsigned long *pRGB = &cloudRGB[0];

    for(int i = 0; i < cloud->points.size();i++,pX++,pY++,pZ++,pRGB++) {
        cloudtmp->points[i].x = (*pX);
        cloudtmp->points[i].y = (*pY);
        cloudtmp->points[i].z = (*pZ);
        cloudtmp->points[i].rgba = (*pRGB);
    }



    /*pcl::FastBilateralFilter<PointT> filter;
    filter.setSigmaS (15.0f);
    filter.setSigmaR (0.05f);
    //filter.setEarlyDivision (false);
    filter.setInputCloud (cloudtmp);
    filter.filter(*cloud_filtered);*/




    // Get Poisson result
    /*pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloudtmp);
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

    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(trianglesSimpl);*/

    PointCloudT::Ptr holder (new PointCloudT);
    PCLViewer::voxelGridFilter(cloudtmp, holder);
    PCLViewer::cloudSmooth(holder, output);
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    PCLViewer::polygonateCloud(holder, triangles);


    //pcl::PolygonMesh::Ptr trianglesPtr(&triangles);
    //triangles = PCLViewer::smoothMesh(trianglesPtr);




    meshViewer->removePolygonMesh("mesh");
    //meshViewer->addPointCloud(output,"smoothed");
    meshViewer->addPolygonMesh(*triangles, "mesh");
    printf("Mesh done\n");
    meshViewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG , "mesh" );
    ui->tabWidget->setCurrentIndex(1);
}

void PCLViewer::lastFrameToggled() {
    if (ui->actionShow_captured_frames->isChecked()) {
        viewer->removePointCloud("frame" + std::to_string(clouds.size()-1));
        for (size_t i = 0; i < clouds.back()->points.size(); i ++) {
            clouds.back()->points[i].a = 50;
        }
        viewer->addPointCloud(clouds.back(), "frame" + std::to_string(clouds.size()));
        //TODO move camera regarding to position in real world, if is it possible
        ui->qvtkWidget->update ();
    }
    else {
        viewer->removePointCloud("frame" + std::to_string(clouds.size()));
        ui->qvtkWidget->update ();
    }
}

pcl::PolygonMesh PCLViewer::smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth) {
    //!!! getting double free or corruption (out)
    std::cout<<"Smoothing mesh\n";
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(meshToSmooth);
    vtk.setNumIter(20000);
    vtk.setConvergence(0.0001);
    vtk.setRelaxationFactor(0.0001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI/5);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);
    return output;
}

void PCLViewer::polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    printf("Marching cubes\n");
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

    cout << "begin marching cubes reconstruction" << endl;

    pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> mc;
    //pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    mc.setInputCloud (cloud_with_normals);
    mc.setSearchMethod (tree);
    mc.reconstruct (*triangles);

    cout << triangles->polygons.size() << " triangles created" << endl;
    //return triangles;
}


void PCLViewer::polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles) {
    std::cout<<"Greedy polygonation\n";


    std::ifstream config_file("config.json");

    if (!config_file.fail()) {
        std::cout << "Config file loaded\n";
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
            if (array_element.first == "greedyProjection")
                std::cout << "greedyProjection" << "\n";
            for (auto & property: array_element.second) {
                if (array_element.first == "greedyProjection")
                    std::cout << " "<< property.first << " = " << property.second.get_value < std::string > () << "\n";
            }
        }

        GPsearchRadius = pt.get<float>("greedyProjection.searchRadius");
        GPmu = pt.get<float>("greedyProjection.mu");
        GPmaximumNearestNeighbors = pt.get<int>("greedyProjection.maximumNearestNeighbors");
    }




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
    gp.setSearchRadius(GPsearchRadius);
    gp.setMu(GPmu);
    gp.setMaximumNearestNeighbors (GPmaximumNearestNeighbors);
    gp.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp.setMinimumAngle(M_PI/18); // 10 degrees
    gp.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp.setNormalConsistency(false);


    gp.setInputCloud (cloud_normals);
    gp.setSearchMethod (tree_normal);
    gp.reconstruct (*triangles);
    std::cout << "Polygons created: " << triangles->polygons.size() << "\n";
    //return triangles;
}

void PCLViewer::voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered) {
    std::cout<<"downsampling filter\n";

    std::ifstream config_file("config.json");

    if (!config_file.fail()) {
        std::cout << "Config file loaded\n";
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
            if (array_element.first == "gridFilter")
                std::cout << "gridFilter" << "\n";
            for (auto & property: array_element.second) {
                if (array_element.first == "gridFilter")
                    std::cout << " "<< property.first << " = " << property.second.get_value < std::string > () << "\n";
            }
        }

        VGFleafSize = pt.get<float>("gridFilter.leafSize");
    }

    pcl::VoxelGrid<PointT> ds;  //create downsampling filter
    ds.setInputCloud (cloudToFilter);
    ds.setLeafSize (VGFleafSize, VGFleafSize, VGFleafSize);
    ds.filter (*filtered);
    std::cout<<"Filtered points: " << filtered->points.size() << "\n";
}

void PCLViewer::cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output) {
    std::cout<<"smoothing "<< cloudToSmooth->points.size() <<" points\n";
    // final version will load from json while startup and changes will be done in GUI


    std::ifstream config_file("config.json");


    if (!config_file.fail()) {
        std::cout << "Config file loaded\n";
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
            if (array_element.first == "mls")
                std::cout << "mls" << "\n";
            for (auto & property: array_element.second) {
                if (array_element.first == "mls")
                    std::cout << " "<< property.first << " = " << property.second.get_value < std::string > () << "\n";
            }
        }

        MLSpolynomialOrder = pt.get<int>("mls.polynomialOrder");
        MLSusePolynomialFit = pt.get<bool>("mls.usePolynomialFit");
        MLSsearchRadius = pt.get<double>("mls.searchRadius");
        MLSsqrGaussParam = pt.get<double>("mls.sqrGaussParam");
        MLSupsamplingRadius = pt.get<double>("mls.upsamplingRadius");
        MLSupsamplingStepSize = pt.get<double>("mls.upsamplingStepSize");
        MLSdilationIterations = pt.get<int>("mls.dilationIterations");
        MLSdilationVoxelSize = pt.get<double>("mls.dilationVoxelSize");
        MLScomputeNormals = pt.get<bool>("mls.computeNormals");
    }



     pcl::MovingLeastSquares<PointT, PointT> mls;
     mls.setInputCloud (cloudToSmooth);
     mls.setSearchRadius (MLSsearchRadius);
     mls.setSqrGaussParam (MLSsqrGaussParam);
     mls.setPolynomialFit (MLSusePolynomialFit);
     mls.setPolynomialOrder (MLSpolynomialOrder);

     //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
     //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
     mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
     //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointXYZRGB>::NONE);
     mls.setPointDensity ( int (60000 * MLSsearchRadius)); // 300 points in a 5 cm radius
     mls.setUpsamplingRadius (MLSupsamplingRadius);
     mls.setUpsamplingStepSize (MLSupsamplingStepSize);
     mls.setDilationIterations (MLSdilationIterations);
     mls.setDilationVoxelSize (MLSdilationVoxelSize);

     pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
     //pcl::search::OrganizedNeighbor<PointT> tree (new pcl::search::OrganizedNeighbor<PointT> ());
     mls.setSearchMethod (tree);
     mls.setComputeNormals (MLScomputeNormals);
     mls.process (*output);
     std::cout<<"now we have "<< output->points.size() <<" points\n";

}

void PCLViewer::closing() {
    //printf("Exiting...\n");
    //interface->stop();
}

PCLViewer::~PCLViewer ()
{
    printf("Exiting...\n");
    if (sensorConnected) {
        interface->stop();
    }
    delete ui;
    clouds.clear();
    //delete &cloud;
}
