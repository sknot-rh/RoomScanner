#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <cstddef>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <QTimer>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>

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

    //OpenNIGrabber
    interface = new pcl::OpenNIGrabber();

    //Setting up UI
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    //Create callback for openni grabber
    boost::function<void (const PointCloudT::ConstPtr&)> f = boost::bind (&PCLViewer::cloud_cb_, this, _1);
    interface->registerCallback(f);
    interface->start ();
    tmrTimer->start(20); // msec
    stream = true;
    stop = false;


    //Connect reset button
    connect(ui->pushButton_reset, SIGNAL (clicked ()), this, SLOT (resetButtonPressed ()));

    // Connect point size slider
    connect(ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    //Connect checkbox
    connect(ui->checkBox, SIGNAL(clicked(bool)), this, SLOT(toggled(bool)));

    //Add empty pointcloud
    viewer->addPointCloud(cloud, "cloud");
    viewer->addPointCloud(key_cloud, "keypoints");


    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
    pSliderValueChanged (2);
}


void PCLViewer::drawFrame() {
    if (!copying && !stop) {
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
        }


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
        viewer->updatePointCloud(cloud ,"cloud");
        ui->qvtkWidget->update ();
    }

}

void PCLViewer::cloud_cb_ (const PointCloudT::ConstPtr &ncloud) {

    if (stream) {
        while(copying) {
            usleep(1);
        }
        copying = true;

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
        copying = false;
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

void PCLViewer::resetButtonPressed ()
{
    viewer->resetCamera();
    ui->qvtkWidget->update();
}


void PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
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
    //delete &cloud;
}
