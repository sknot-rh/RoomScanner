#include "application.h"
#include "../build/ui_application.h"
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <QTimer>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <QFileDialog>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <QMessageBox>
#include <QMovie>
#include <unistd.h>
#include <pcl/console/parse.h>

parameters* parameters::instance = 0;

RoomScanner::RoomScanner (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::RoomScanner) {
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
        boost::function<void (const PointCloudAT::ConstPtr&)> f = boost::bind (&RoomScanner::cloud_cb_, this, _1);
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

    //Connect poly button
    connect(ui->pushButton_stream, SIGNAL (clicked ()), this, SLOT (streamButtonPressed ()));


    //Add empty pointclouds
    viewer->addPointCloud(kinectCloud, "kinectCloud");

    //viewer->addPointCloud(key_cloud, "keypoints");

    //viewer->setBackgroundColor(0.5f, 0.5f, 0.5f);

    pSliderValueChanged (2);
    ui->tabWidget->setCurrentIndex(0);

    /*mytemplate test;
    test.mytemplateMethod(5);*/

}

void RoomScanner::drawFrame() {
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
            for(int i = 0; i < kinectCloud->points.size(); i++,pX++,pY++,pZ++,pRGB++) {
                kinectCloud->points[i].x = (*pX);
                kinectCloud->points[i].y = (*pY);
                kinectCloud->points[i].z = (*pZ);
                kinectCloud->points[i].rgba = (*pRGB);
                //cloud->points[i].a = 128; //for better stitching?
            }
            mtx_.unlock();
        }

        if (ui->actionShow_keypoints->isChecked() == true) {
            parameters* params = parameters::GetInstance();
            // Downsample data for faster computation
            PointCloudT::Ptr tmp;
            tmp.reset(new PointCloudT);
            filters::downsample(kinectCloud, *tmp, 0.05);

            // Estimate the sift interest points using Intensity values from RGB values
            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
            sift.setSearchMethod(tree);
            sift.setScales(params->SIFTmin_scale, params->SIFTn_octaves, params->SIFTn_scales_per_octave);
            sift.setMinimumContrast(params->SIFTmin_contrast);
            sift.setInputCloud(tmp);
            sift.compute(result);

            copyPointCloud(result, *key_cloud); // from PointWithScale to PointCloudAT

            for (int var = 0; var < key_cloud->size(); ++var) {
                key_cloud->points[var].r = 0;
                key_cloud->points[var].g = 255;
                key_cloud->points[var].b = 0;
            }
            viewer->updatePointCloud(key_cloud,"keypoints");
        }
        viewer->updatePointCloud(kinectCloud,"kinectCloud");
        ui->qvtkWidget->update ();
    }
}

void RoomScanner::cloud_cb_ (const PointCloudAT::ConstPtr &ncloud) {
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
            for (int j = 0; j<ncloud->height; j++) {
                for (int i = 0; i<ncloud->width; i++,pX++,pY++,pZ++,pRGB++) {
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

void RoomScanner::toggled(bool value) {
    if (value) {
        viewer->addCoordinateSystem(1, 0, 0, 0, "viewer", 0);
    }
    else {
        viewer->removeCoordinateSystem("viewer", 0);
    }
    ui->qvtkWidget->update();
}

void RoomScanner::resetButtonPressed() {
    viewer->resetCamera();
    ui->qvtkWidget->update();
}

void RoomScanner::saveButtonPressed() {
    if (!sensorConnected) {
        return;
    }

    PointCloudT::Ptr tmp (new PointCloudT);
    PointCloudT::Ptr output (new PointCloudT);
    stream = false; // "safe" copy
    // Allocate enough space and copy the basics
    tmp->header   = kinectCloud->header;
    tmp->width    = kinectCloud->width;
    tmp->height   = kinectCloud->height;
    tmp->is_dense = kinectCloud->is_dense;
    tmp->sensor_orientation_ = kinectCloud->sensor_orientation_;
    tmp->sensor_origin_ = kinectCloud->sensor_origin_;
    tmp->points.resize (kinectCloud->points.size ());

    memcpy (&tmp->points[0], &kinectCloud->points[0], kinectCloud->points.size () * sizeof (PointT));
    stream = true;

    output = tmp;
    //filters::voxelGridFilter(tmp, output, 0.001); //global downsampling
    std::vector<int> indices;
    removeNaNFromPointCloud(*output,*output, indices);
    filters::oultlierRemoval(output, output, 0.8f);
    //filters::oultlierRemoval(output, output);
    clouds.push_back(output);

    std::cout << "Saving frame #"<<clouds.size()<<"\n";

    std::stringstream ss;
    ss << "frame_" << clouds.size()<<  ".pcd";
    std::string s = ss.str();

    pcl::io::savePCDFile (s, *output);
    lastFrameToggled();
}

void RoomScanner::pSliderValueChanged (int value)
{
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "kinectCloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloudFromFile");
    ui->lcdNumber_p->display(value);
    ui->qvtkWidget->update ();
}

void RoomScanner::loadActionPressed() {
    QString fileName = QFileDialog::getOpenFileName(this,
                       tr("Choose Point Cloud"), ".", tr("Point Cloud Files (*.pcd)"));
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
        viewer->updatePointCloud(cloudFromFile,"cloudFromFile");
    clouds.push_back(cloudFromFile);
    ui->qvtkWidget->update();
}

void RoomScanner::polyButtonPressed() {

    if (clouds.empty()) {
        if (sensorConnected) {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::polyButtonPressedFunc, this));
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
        if (!registered && clouds.size() > 1) {
            QMessageBox::warning(this, "Warning", "Pointclouds ready to registrate!");
            std::cerr << "Pointclouds ready to registrate!\n";
            return;
        }
        else {
            ui->tabWidget->setCurrentIndex(1);
            boost::thread* thr2 = new boost::thread(boost::bind(&RoomScanner::polyButtonPressedFunc, this));
            labelPolygonate = new QLabel;
            loading(labelPolygonate);
        }
    }
}


void RoomScanner::loading(QLabel* label) {
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

void RoomScanner::polyButtonPressedFunc() {
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

            for(int i = 0; i < kinectCloud->points.size(); i++,pX++,pY++,pZ++,pRGB++) {
                cloudtmp->points[i].x = (*pX);
                cloudtmp->points[i].y = (*pY);
                cloudtmp->points[i].z = (*pZ);
                cloudtmp->points[i].rgba = (*pRGB);
            }
            //mtx_.unlock();

            //}

            //filters::voxelGridFilter(cloudtmp, output);
            //filters::cloudSmooth(holder, output); TODO!!!
            mesh::polygonateCloud(cloudtmp, triangles);
        }
        else {
            // empty clouds & no sensor
            QMessageBox::warning(this, "Error", "No pointcloud to polygonate!");
            std::cerr << "No cloud to polygonate!\n";
            return;

        }
    }
    else {
        //filters::voxelGridFilter(clouds.back(), output);
        //filters::cloudSmooth(holder, output);
        mesh::polygonateCloud(clouds.back(), triangles);
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
    meshViewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, "mesh" );
    ui->qvtkWidget_2->update();

}

void RoomScanner::lastFrameToggled() {
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

void RoomScanner::closing() {
    //printf("Exiting...\n");
    //interface->stop();
}

RoomScanner::~RoomScanner ()
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

void RoomScanner::actionClearTriggered()
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

void RoomScanner::regButtonPressed() {
    if (clouds.size() < 2) {
        std::cerr << "To few clouds to registrate!\n";
    }
    //stop = true;
    stream = false;
    std::cout << "Registrating " << clouds.size() << " point clouds.\n";
    labelRegistrate = new QLabel;
    boost::thread* thr = new boost::thread(boost::bind(&RoomScanner::registrateNClouds, this));
    loading(labelRegistrate);
}

PointCloudT::Ptr RoomScanner::registrateNClouds() {
    PointCloudT::Ptr result (new PointCloudT);
    PointCloudT::Ptr globalResult (new PointCloudT);
    PointCloudT::Ptr source, target;

    registration reg;
    connect(&reg, SIGNAL(regFrameSignal()), this, SLOT(regFrameSlot()));

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    viewer->removeAllPointClouds();
    viewer->addPointCloud(clouds[1], "target");
    viewer->addPointCloud(clouds[0], "source");
    for (int i = 1; i < clouds.size(); i++) {

        source = clouds[i-1];
        pcl::io::savePCDFileBinary ("previousRegisteredOutput.pcd", *(source));
        target = clouds[i];
        viewer->updatePointCloud(target, "target");

        // estimated source position done with fpfh features
        reg.computeTransformation(source, target);

        // it would be nice to visualize progress, but this method runs in new thread so it has to be investigated
        /*viewer->addPointCloud(source,"source");
        viewer->addPointCloud(target,"target");
        viewer->spinOnce();*/

        PointCloudT::Ptr temp (new PointCloudT);
        reg.pairAlign (source, target, temp, pairTransform, true);
        //pcl::transformPointCloud (*temp, *result, GlobalTransform);
        pcl::transformPointCloud (*temp, *result, GlobalTransform);
        ui->qvtkWidget->update();

        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;

        //clouds[i] = result;
        filters::voxelGridFilter(result, clouds[i]);
    }



    copyPointCloud(*(clouds.back()), *(clouds.front()));
    clouds.erase(clouds.begin()+1, clouds.end());
    pcl::io::savePCDFileBinary ("registeredOutput.pcd", *(clouds[0]));

    std::cout << "Registrated Point Cloud has " << clouds[clouds.size()-1]->points.size() << " points.\n";
    labelRegistrate->close();
    registered = true;
    return result;
}



void RoomScanner::tabChangedEvent(int tabIndex) {
    if (tabIndex == 0) {
        stream = true;
    }
    else {
        std::cout<<"Stopping stream...\n";
        stream = false;
    }
}

void RoomScanner::keypointsToggled() {
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


void RoomScanner::regFrameSlot() {
    printf("Signal receieved\n");
    viewer->updatePointCloud(registration::regFrame, "source");
    ui->qvtkWidget->update();
    viewer->resetCamera();
}

void RoomScanner::streamButtonPressed() {
    stream = true;
    viewer->removeAllPointClouds();
    viewer->addPointCloud(kinectCloud, "kinectCloud");
    ui->qvtkWidget->update();
    viewer->resetCamera();
}





