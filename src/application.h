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

#ifndef APPLICATION_H
#define APPLICATION_H

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
#include <QMetaObject>
#include <QDebug>


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
#include <pcl/console/time.h>
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
#include "clicklabel.h"

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
    void loading(clickLabel* label);
    void registerNClouds();
    void saveButtonPressedFun();
    void smoothAction();
    //void loadActionPressedFun();
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

signals:
    void closeLabelSignal(int);
    void resetCameraSignal();

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

    void closeLabelSlot(int index);

    void resetCameraSlot();

    void saveRegFrame();

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
    clickLabel* labelRegister;
    int LREG = 0;
    clickLabel* labelPolygonate;
    int LPOL = 1;
    clickLabel* labelSave;
    int LSAV = 2;
    clickLabel* labelSmooth;
    int LSMO = 3;
    clickLabel* labelLoad;
    int LLOA = 4;

};



#endif // APPLICATION_H
