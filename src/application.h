#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <unistd.h>

// Qt
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QTimer>
#include <QMessageBox>
#include <QMovie>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/io/point_cloud_image_extractors.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Boost
#include <boost/atomic.hpp>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <boost/thread/thread.hpp>

#include "parameters.h"
#include "types.h"
#include "filters.h"
#include "pointrepr.h"
#include "mesh.h"
#include "registration.h"
#include "texturing.h"

namespace Ui
{
    class RoomScanner;
}

class RoomScanner : public QMainWindow
{
    Q_OBJECT
    QThread thread;

public:
    RoomScanner (QWidget *parent = 0);
    ~RoomScanner ();
    void cloud_cb_ (const PointCloudAT::ConstPtr &ncloud);
    void cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
    void polyButtonPressedFunc();
    void loading(QLabel* label);
    void registrateNClouds();
    void saveButtonPressedFun();
    void smoothAction();
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

public slots:
    void resetButtonPressed(void);

    void saveButtonPressed(void);

    void polyButtonPressed(void);

    void regButtonPressed(void);

    void drawFrame(void);

    void coordSysToggled(bool value);

    void loadActionPressed(void);

    void lastFrameToggled(void);

    void actionClearTriggered(void);

    void tabChangedEvent(int);

    void keypointsToggled(void);

    void regFrameSlot(void);

    void streamButtonPressed(void);

    void actionSmoothTriggered(void);

    void loadConfigFile();

    void refreshParams();

    void saveModelButtonPressed();

    void actionQuitTriggered(void);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> meshViewer;
    PointCloudT::Ptr kinectCloud;
    PointCloudT::Ptr registratedCloud;
    PointCloudAT::Ptr key_cloud;
    PointCloudT::Ptr regResult;
    std::vector<PointCloudT::Ptr> clouds;
    std::vector<std::string> images;
    QTimer *tmrTimer;
    QMovie *movie;
    pcl::PolygonMesh::Ptr triangles;

    unsigned int red;
    unsigned int green;
    unsigned int blue;
    pcl::Grabber* interface;
    boost::atomic<bool> stream;
    bool copying;
    bool stop;
    int cloudWidth;
    int cloudHeight;
    std::vector<float> cloudX, cloudY, cloudZ;
    std::vector<unsigned long> cloudRGB;
    boost::mutex mtx_;
    bool sensorConnected;
    bool registered = false;


private:
    Ui::RoomScanner *ui;
    QLabel* labelRegistrate;
    QLabel* labelPolygonate;
    QLabel* labelSave;
    QLabel* labelSmooth;

};

#endif // PCLVIEWER_H
