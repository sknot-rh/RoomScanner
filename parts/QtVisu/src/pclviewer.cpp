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

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer) {
    ui->setupUi (this);
    this->setWindowTitle ("RoomScanner");

    // Timer for cloud & UI update
    QTimer *tmrTimer = new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(drawFrame()));

    //Create empty clouds
    cloud.reset(new PointCloudT);
    key_cloud.reset(new PointCloudT);


    //Tell to sensor in which position is expected input
    Eigen::Quaternionf m;
    m = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(0.0f,  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ());
    cloud->sensor_orientation_ = m;
    key_cloud->sensor_orientation_ = m;

    copying = stream = false;
    bool sensorConnected = false;

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
        boost::function<void (const PointCloudT::ConstPtr&)> f = boost::bind (&PCLViewer::cloud_cb_, this, _1);
        interface->registerCallback(f);
        interface->start ();
    }
    tmrTimer->start(20); // msec
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
            pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
            pcl::PointCloud<pcl::PointWithScale> result;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
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

void PCLViewer::cloud_cb_ (const PointCloudT::ConstPtr &ncloud) {

    if (stream) {
        /*while(copying) {
            usleep(1);
        }*/
        if (mtx_.try_lock()) {
            //copying = true;

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
                    PointT P = ncloud->at(i,j);
                    (*pX) = P.x;
                    (*pY) = P.y;
                    (*pZ) = P.z;
                    (*pRGB) = P.rgba;
                }
            }
            // Data copied
            //copying = false;
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
    PointCloudT::Ptr cloud2 (new PointCloudT);
    if (pcl::io::loadPCDFile<PointT> (utf8_fileName, *cloud2) == -1) //* load the file
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
    // Load input file into a PointCloud<T> with an appropriate type
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudtmp (new pcl::PointCloud<pcl::PointXYZRGB>);
     cloudtmp->clear();
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


     //Normal Estimation
     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normEstim;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
     tree->setInputCloud(cloudtmp);
     normEstim.setInputCloud(cloudtmp);
     normEstim.setSearchMethod(tree);
     normEstim.setKSearch(20);
     normEstim.compute(*normals);

     //Concatenate the cloud with the normal fields
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
     pcl::concatenateFields(*cloudtmp,*normals,*cloud_normals);

     //Create  search tree to include cloud with normals
     pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_normal (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
     tree_normal->setInputCloud(cloud_normals);

     //Initialize objects for triangulation
     pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp;
     pcl::PolygonMesh triangles;

     //Max distance between connecting edge points
     gp.setSearchRadius(0.05);
     gp.setMu(2.5);
     gp.setMaximumNearestNeighbors (100);
     gp.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
     gp.setMinimumAngle(M_PI/18); // 10 degrees
     gp.setMaximumAngle(2*M_PI/3); // 120 degrees
     gp.setNormalConsistency(false);

     // Get result
     gp.setInputCloud (cloud_normals);
     gp.setSearchMethod (tree_normal);
     gp.reconstruct (triangles);
     meshViewer->removePolygonMesh();
     meshViewer->addPolygonMesh(triangles);
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

void PCLViewer::closing() {
    //printf("Exiting...\n");
    //interface->stop();
}

PCLViewer::~PCLViewer ()
{
    printf("Exiting...\n");
    interface->stop();
    delete ui;
    clouds.clear();
    //delete &cloud;
}
