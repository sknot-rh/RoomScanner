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
#include <pcl/io/pcd_io.h>
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
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <QMessageBox>
#include <QMovie>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp_nl.h>
#include <unistd.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>


PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer) {
    ui->setupUi (this);
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    this->setWindowTitle ("RoomScanner");
    this->setWindowIcon(QIcon(":/images/Terminator.jpg"));
    movie = new QMovie(":/images/box.gif");

    // Timer for cloud & UI update
    tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(drawFrame()));

    //Create empty clouds
    kinectCloud.reset(new PointCloudT);
    key_cloud.reset(new PointCloudAT);


    //Tell to sensor in which position is expected input
    Eigen::Quaternionf m;
    m = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(0.0f,  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ());
    kinectCloud->sensor_orientation_ = m;
    key_cloud->sensor_orientation_ = m;

    copying = stream = false;
    sensorConnected = false;
    registered = false;

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
    //stop = false;

    //Connect reset button
    connect(ui->pushButton_reset, SIGNAL (clicked ()), this, SLOT (resetButtonPressed ()));

    //Connect save button
    connect(ui->pushButton_save, SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));

    //Connect poly button
    connect(ui->pushButton_poly, SIGNAL (clicked ()), this, SLOT (polyButtonPressed ()));

    //Connect poly button
    connect(ui->pushButton_reg, SIGNAL (clicked ()), this, SLOT (regButtonPressed ()));

    //Connect point size slider
    connect(ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    //Connect checkbox
    connect(ui->checkBox, SIGNAL(clicked(bool)), this, SLOT(toggled(bool)));

    //Connect menu checkbox - show last frame
    connect(ui->actionShow_captured_frames, SIGNAL(triggered()), this, SLOT(lastFrameToggled()));

    //Connect load action
    connect(ui->actionLoad_Point_Cloud, SIGNAL (triggered()), this, SLOT (loadActionPressed ()));

    //Connect clear action
    connect(ui->actionClear, SIGNAL (triggered()), this, SLOT (actionClearTriggered ()));

    //Connect tab change action
    connect(ui->tabWidget, SIGNAL (currentChanged(int)), this, SLOT (tabChangedEvent(int)));

    //Connect keypoint action
    connect(ui->actionShow_keypoints, SIGNAL(triggered()), this, SLOT(keypointsToggled()));


    //Add empty pointclouds
    viewer->addPointCloud(kinectCloud, "kinectCloud");

    //viewer->addPointCloud(key_cloud, "keypoints");

    //viewer->setBackgroundColor(0.5f, 0.5f, 0.5f);

    pSliderValueChanged (2);
    ui->tabWidget->setCurrentIndex(0);

}

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


void PCLViewer::drawFrame() {
    if (stream) {
        if (mtx_.try_lock()) {
            kinectCloud->clear();
            kinectCloud->width = cloudWidth;
            kinectCloud->height = cloudHeight;
            kinectCloud->points.resize(cloudHeight*cloudWidth);
            kinectCloud->is_dense = false;
            // Fill cloud
            float *pX = &cloudX[0];
            float *pY = &cloudY[0];
            float *pZ = &cloudZ[0];
            unsigned long *pRGB = &cloudRGB[0];
            for(int i = 0; i < kinectCloud->points.size();i++,pX++,pY++,pZ++,pRGB++) {
                kinectCloud->points[i].x = (*pX);
                kinectCloud->points[i].y = (*pY);
                kinectCloud->points[i].z = (*pZ);
                kinectCloud->points[i].rgba = (*pRGB);
                //cloud->points[i].a = 128; //for better stitching?
            }
            mtx_.unlock();
        }


        if (ui->actionShow_keypoints->isChecked() == true) {
            // Estimate the sift interest points using Intensity values from RGB values
            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
            sift.setSearchMethod(tree);
            sift.setScales(min_scale, n_octaves, n_scales_per_octave);
            sift.setMinimumContrast(min_contrast);
            sift.setInputCloud(kinectCloud);
            sift.compute(result);

            copyPointCloud(result, *key_cloud);

            for (int var = 0; var < key_cloud->size(); ++var) {
                key_cloud->points[var].r = 0;
                key_cloud->points[var].g = 255;
                key_cloud->points[var].b = 0;

            }

            viewer->updatePointCloud(key_cloud ,"keypoints");

        }
        viewer->updatePointCloud(kinectCloud ,"kinectCloud");
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
    if (!sensorConnected) {
        return;
    }

    PointCloudT::Ptr cloud_out (new PointCloudT);
    stream = false; // "safe" copy
    // Allocate enough space and copy the basics
     cloud_out->header   = kinectCloud->header;
     cloud_out->width    = kinectCloud->width;
     cloud_out->height   = kinectCloud->height;
     cloud_out->is_dense = kinectCloud->is_dense;
     cloud_out->sensor_orientation_ = kinectCloud->sensor_orientation_;
     cloud_out->sensor_origin_ = kinectCloud->sensor_origin_;
     cloud_out->points.resize (kinectCloud->points.size ());

    memcpy (&cloud_out->points[0], &kinectCloud->points[0], kinectCloud->points.size () * sizeof (PointT));
    stream = true;
    clouds.push_back(cloud_out);
    std::cout << "Saving frame n"<<clouds.size()<<"\n";

    std::stringstream ss;
    ss << "bla" << clouds.size()<<  ".pcd";
    std::string s = ss.str();


    pcl::io::savePCDFile (s , *cloud_out);
    //pcl::io::savePCDFile ("bljjj0.pcd" , cloud_out);

    lastFrameToggled();
}

void PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "kinectCloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloudFromFile");
  ui->lcdNumber_p->display(value);
  ui->qvtkWidget->update ();
}

void PCLViewer::loadActionPressed() {
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Choose Point Cloud"), "/home", tr("Point Cloud Files (*.pcd)"));
    std::string utf8_fileName = fileName.toUtf8().constData();
    PointCloudT::Ptr cloudFromFile (new PointCloudT);
    if (pcl::io::loadPCDFile<PointT> (utf8_fileName, *cloudFromFile) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read pcd file!\n");
        return;
    }
    for (size_t i = 0; i < cloudFromFile->size(); i++)
    {
        cloudFromFile->points[i].a = 255;
    }

    ui->tabWidget->setCurrentIndex(0);
    std::cout << "PC Loaded from file " << utf8_fileName << "\n";
    viewer->removeAllPointClouds();
    if (!viewer->addPointCloud(cloudFromFile, "cloudFromFile"))
        viewer->updatePointCloud(cloudFromFile ,"cloudFromFile");
    clouds.push_back(cloudFromFile);
    ui->qvtkWidget->update();
}

void PCLViewer::polyButtonPressed() {

    if (clouds.empty()) {
        if (sensorConnected) {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&PCLViewer::polyButtonPressedFunc, this));
            labelPolygonate = new QLabel;
            loading(labelPolygonate);
        }
        else {
            // empty clouds & no sensor
            QMessageBox::warning(this, "Error", "No pointcloud to polygonate!");
            std::cerr << "No cloud to polygonate!\n";
            return;
        }
    }
    else {
        if (!registered) {
            QMessageBox::warning(this, "Warning", "Pointclouds ready to registrate!");
            std::cerr << "Pointclouds ready to registrate!\n";
            return;
        }
        else {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&PCLViewer::polyButtonPressedFunc, this));
            labelPolygonate = new QLabel;
            loading(labelPolygonate);
        }

    }
}


void PCLViewer::loading(QLabel* label) {


    if (!movie->isValid()) {
        std::cerr<<"Invalid loading image " << movie->fileName().toStdString() << "\n";
        return;
    }
    label->setMovie(movie);
    label->setFixedWidth(200);
    label->setFixedHeight(200);
    label->setFrameStyle(QFrame::NoFrame);
    label->setAttribute(Qt::WA_TranslucentBackground);
    label->setWindowModality(Qt::ApplicationModal);
    label->setContentsMargins(0,0,0,0);
    label->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    label->show();
    movie->start();
}

void PCLViewer::polyButtonPressedFunc() {
    PointCloudT::Ptr cloudtmp (new PointCloudT);
    PointCloudT::Ptr output (new PointCloudT);
    PointCloudT::Ptr holder (new PointCloudT);
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    //stop stream to avoid changes of polygonated kinect point cloud

    //                                                     /+++ polygonate kinect frame
    //                               /+++ sensor connected?
    // polygonation --- empty clouds?                      \--- error           /+++ polygonate cloud
    //                               \--- non empty clouds --- registered cloud?
    //                                                                          \--- register & polygonate?
    //

    if (clouds.empty()) {
        if (sensorConnected) {

            //if (mtx_.try_lock()) {
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

                for(int i = 0; i < kinectCloud->points.size();i++,pX++,pY++,pZ++,pRGB++) {
                    cloudtmp->points[i].x = (*pX);
                    cloudtmp->points[i].y = (*pY);
                    cloudtmp->points[i].z = (*pZ);
                    cloudtmp->points[i].rgba = (*pRGB);
                }
                //mtx_.unlock();

            //}

            PCLViewer::voxelGridFilter(cloudtmp, output);
            //PCLViewer::cloudSmooth(holder, output);
            PCLViewer::polygonateCloud(output, triangles);
        }
        else {
            // empty clouds & no sensor
            QMessageBox::warning(this, "Error", "No pointcloud to polygonate!");
            std::cerr << "No cloud to polygonate!\n";
            return;

        }
    }
    else {
            PCLViewer::voxelGridFilter(clouds.back(), output);
            //PCLViewer::cloudSmooth(holder, output);
            PCLViewer::polygonateCloud(output, triangles);
    }


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





    //pcl::PolygonMesh::Ptr trianglesPtr(&triangles);
    //triangles = PCLViewer::smoothMesh(trianglesPtr);


    labelPolygonate->close();
    meshViewer->removePolygonMesh("mesh");
    //meshViewer->addPointCloud(output,"smoothed");
    meshViewer->addPolygonMesh(*triangles, "mesh");
    printf("Mesh done\n");
    meshViewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG , "mesh" );
    ui->qvtkWidget_2->update();

}

void PCLViewer::lastFrameToggled() {
    if (clouds.empty()) {
        return;
    }
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

    std::cout << "begin marching cubes reconstruction" << std::endl;

    pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> mc;
    //pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    mc.setInputCloud (cloud_with_normals);
    mc.setSearchMethod (tree);
    mc.reconstruct (*triangles);

    std::cout << triangles->polygons.size() << " triangles created" << std::endl;
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
    //pcl::UniformSampling<PointT> ds;  //create downsampling filter
    ds.setInputCloud (cloudToFilter);
    ds.setLeafSize (VGFleafSize, VGFleafSize, VGFleafSize);
    //ds.setRadiusSearch(VGFleafSize);
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
    tmrTimer->stop();
    //delete &cloud;
}

void PCLViewer::actionClearTriggered()
{
    clouds.clear();
    viewer->removeAllPointClouds();
    meshViewer->removeAllPointClouds();
    ui->qvtkWidget->update();
    ui->qvtkWidget_2->update();
    if (sensorConnected) {
        viewer->addPointCloud(kinectCloud, "kinectCloud");
    }
    ui->tabWidget->setCurrentIndex(0);
    stream = true;
    registered = false;
}

void PCLViewer::regButtonPressed() {
    if (clouds.size() < 2) {
        std::cerr << "To few clouds to registrate!\n";
    }
    //stop = true;
    stream = false;
    std::cout << "Registrating " << clouds.size() << " point clouds.\n";
    labelRegistrate = new QLabel;
    boost::thread* thr = new boost::thread(boost::bind(&PCLViewer::registrateNClouds, this));
    loading(labelRegistrate);
}

PointCloudT::Ptr PCLViewer::registrateNClouds() {
    PointCloudT::Ptr result (new PointCloudT);
    PointCloudT::Ptr globalResult (new PointCloudT);
    PointCloudT::Ptr source, target;

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;


    for (int i = 1; i < clouds.size(); i++) {

        source = clouds[i-1];
        target = clouds[i];

        /*// Create the filtering object
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (clouds[i-1]);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*source);

        // Create the filtering object 2
        pcl::StatisticalOutlierRemoval<PointT> sor2;
        sor2.setInputCloud (clouds[i]);
        sor2.setMeanK (50);
        sor2.setStddevMulThresh (1.0);
        sor2.filter (*target);*/

        PointCloudT::Ptr temp (new PointCloudT);
        PCLViewer::pairAlign (source, target, temp, pairTransform, true);


        //PCLViewer::computeTransformation (source, target, pairTransform);
        /*if (!viewer->addPointCloud(clouds[i-1], "source")) {
            viewer->updatePointCloud(clouds[i-1], "source");
        }
        if (!viewer->addPointCloud(clouds[i], "target")) {
            viewer->updatePointCloud(clouds[i], "target");
        }*/

        //transform current pair into the global transform
        pcl::transformPointCloud (*temp, *result, GlobalTransform);


        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;

        //clouds[i] = result;
        //TODO!!! filtrovat vysledek
        pcl::UniformSampling<PointT> uniform;
        uniform.setRadiusSearch (0.01);  // 1cm

        uniform.setInputCloud (result);
        uniform.filter (*clouds[i]);

    }



    /*source = clouds[0];
    target = clouds[1];

    Eigen::Matrix4f transform;
      PCLViewer::computeTransformation (source, target, transform);

      std::cerr << transform << std::endl;
      // Transform the data and write it to disk
      pcl::PointCloud<PointT> output;
      pcl::transformPointCloud (*source, output, transform);
      pcl::io::savePCDFileBinary ("kokos.pcd", output);*/

    //result = clouds[clouds.size()-1];
    viewer->removeAllPointClouds();
    viewer->addPointCloud(clouds[clouds.size()-1], "registrated");



    std::cout << "Registrated Point Cloud has " << result->points.size() << " points.\n";
    labelRegistrate->close();
    registered = true;
    return result;
}



void PCLViewer::tabChangedEvent(int tabIndex) {
    if (tabIndex == 0) {
        stream = true;
        //stop = false;
    }
    else {
        std::cout<<"Stopping stream...\n";
        stream = false;
        //stop = true;
    }
}

void PCLViewer::keypointsToggled() {
    if (!ui->actionShow_keypoints->isChecked()) {
        viewer->removePointCloud("keypoints");
        ui->qvtkWidget->update();
    }
    else {
        viewer->addPointCloud(key_cloud, "keypoints");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
        ui->qvtkWidget->update();
    }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void PCLViewer::pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloudT::Ptr src (new PointCloudT);
  PointCloudT::Ptr tgt (new PointCloudT);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (1.0);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

        //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

  }

    //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }


void PCLViewer::computeTransformation (const PointCloudT::Ptr &src_origin,
                       const PointCloudT::Ptr &tgt_origin,
                       Eigen::Matrix4f &transform)
{
    std::cout << "computeTransformation\n";
  // Get an uniform grid of keypoints
  PointCloudT::Ptr keypoints_src (new PointCloudT), keypoints_tgt (new PointCloudT);

  PointCloudT::Ptr src (new PointCloudT),
                   tgt (new PointCloudT);

  PCLViewer::downsample (src_origin, tgt_origin, *src, *tgt);

  PCLViewer::estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
  printf ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

  // Compute normals for all points keypoint
  pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>),
                          normals_tgt (new pcl::PointCloud<pcl::Normal>);
  PCLViewer::estimateNormals (src, tgt, *normals_src, *normals_tgt);
  printf ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

  // Compute FPFH features at each keypoint
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src (new pcl::PointCloud<pcl::FPFHSignature33>),
                                   fpfhs_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
  PCLViewer::estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

  // Find correspondences between keypoints in FPFH space
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences),
                     good_correspondences (new pcl::Correspondences);
  PCLViewer::findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

  // Reject correspondences based on their XYZ distance
  PCLViewer::rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

  for (int i = 0; i < good_correspondences->size (); ++i)
    std::cerr << good_correspondences->at (i) << std::endl;
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
}


void PCLViewer::rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const PointCloudT::Ptr &keypoints_src,
                          const PointCloudT::Ptr &keypoints_tgt,
                          pcl::Correspondences &remaining_correspondences)
{
    std::cout << "rejectBadCorrespondences\n";
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputCloud<PointT> (keypoints_src);
  rej.setInputTarget<PointT> (keypoints_tgt);
  rej.setMaximumDistance (1);    // 1m
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}

void PCLViewer::findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                     pcl::Correspondences &all_correspondences)
{
    std::cout << "findCorrespondences\n";
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}

void PCLViewer::estimateFPFH (const PointCloudT::Ptr &src,
              const PointCloudT::Ptr &tgt,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
              const pcl::PointCloud<pcl::Normal>::Ptr &normals_tgt,
              const PointCloudT::Ptr &keypoints_src,
              const PointCloudT::Ptr &keypoints_tgt,
              pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src,
              pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_tgt)
{
    std::cout << "estimateFPFH\n";
  pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (keypoints_src);
  fpfh_est.setInputNormals (normals_src);
  fpfh_est.setRadiusSearch (1); // 1m
  fpfh_est.setSearchSurface (src);
  fpfh_est.compute (fpfhs_src);

  fpfh_est.setInputCloud (keypoints_tgt);
  fpfh_est.setInputNormals (normals_tgt);
  fpfh_est.setSearchSurface (tgt);
  fpfh_est.compute (fpfhs_tgt);
}

void PCLViewer::estimateNormals (const PointCloudT::Ptr &src,
                 const PointCloudT::Ptr &tgt,
                 pcl::PointCloud<pcl::Normal> &normals_src,
                 pcl::PointCloud<pcl::Normal> &normals_tgt)
{
    std::cout << "estimateNormals\n";
  pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
  normal_est.setNumberOfThreads(4);
  normal_est.setInputCloud (src);
  std::cout << "normals source " << src->points.size() << "\n";
  normal_est.setRadiusSearch (0.5);  // 50cm
  normal_est.compute (normals_src);

  std::cout << "normals target " << tgt->points.size() << "\n";
  normal_est.setInputCloud (tgt);
  normal_est.compute (normals_tgt);
}

void PCLViewer::estimateKeypoints (const PointCloudT::Ptr &src,
                   const PointCloudT::Ptr &tgt,
                   PointCloudT &keypoints_src,
                   PointCloudT &keypoints_tgt)
{
    std::cout << "estimateKeypoints\n";
  // Get an uniform grid of keypoints
  pcl::UniformSampling<PointT> uniform;
  uniform.setRadiusSearch (1);  // 1m

  uniform.setInputCloud (src);
  uniform.filter (keypoints_src);

  uniform.setInputCloud (tgt);
  uniform.filter (keypoints_tgt);
}

void PCLViewer::downsample (const PointCloudT::Ptr &src_origin, const PointCloudT::Ptr &tgt_origin, PointCloudT &src, PointCloudT &tgt) {

    std::cout << "downsample\n";
      // Get an uniform grid of keypoints
      pcl::UniformSampling<PointT> uniform;
      uniform.setRadiusSearch (0.05);  // 5cm

      uniform.setInputCloud (src_origin);
      uniform.filter (src);

      uniform.setInputCloud (tgt_origin);
      uniform.filter (tgt);

}







