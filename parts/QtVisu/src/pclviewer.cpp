#include "pclviewer.h"
#include "../build/ui_pclviewer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <cstddef>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>


PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer) {
    ui->setupUi (this);
    this->setWindowTitle ("RoomScanner");

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (200);

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }


    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);


    interface = new pcl::OpenNIGrabber();

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();


    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&PCLViewer::cloud_cb_, this, _1);
    //boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&PCLViewer::cloud_cb_, this, _1);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    //boost::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr&)> f = boost::bind (&PCLViewer::cloud_cb_2, this, _1);

    interface->registerCallback(f);

    interface->start ();
    //#TODO interface stop



    // Connect "random" button and the function
    connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

    // Connect R,G,B sliders and their functions
    connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
    connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
    connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
    connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    viewer->addPointCloud (cloud, "cloud");
    pSliderValueChanged (2);
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}


void PCLViewer::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &ncloud) {
    cloud->points.resize (ncloud->size());
    /*for (size_t i = 0; i < ncloud->size(); i++) {
        //cloud->push_back (pcl::PointXYZRGBA(ncloud->points[i]));
        cloud->points[i].x = ncloud->points[i].x;
        cloud->points[i].y = ncloud->points[i].y;
        cloud->points[i].z = ncloud->points[i].z;
        printf("%u %u %u\n", ncloud->points[i].r, ncloud->points[i].g, ncloud->points[i].b);
        cloud->points[i].r = 128;//(unsigned int)ncloud->points[i].r;
        cloud->points[i].g = 128;//(unsigned int)ncloud->points[i].g;
        cloud->points[i].b = 128;//(unsigned int)ncloud->points[i].b;
        cloud->points[i].a = 128;
    }*/

    if (first) {
        viewer->addPointCloud (ncloud, "cloud2");
        first = false;
    }
    viewer->updatePointCloud (ncloud, "cloud2");


    ui->qvtkWidget->update ();
}


void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
  }

  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::RGBsliderReleased ()
{
  // Set the new color
  for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
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
}
