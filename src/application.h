#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>
#include <QThread>
#include <QLabel>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <boost/atomic.hpp>
#include "parameters.h"
#include "types.h"
#include "filters.h"
#include "pointrepr.h"
#include "mesh.h"
#include "registration.h"

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
    PointCloudT::Ptr registrateNClouds();

public slots:
    void resetButtonPressed(void);

    void saveButtonPressed(void);

    void polyButtonPressed(void);

    void regButtonPressed(void);

    void pSliderValueChanged(int value);

    void closing(void);

    void drawFrame(void);

    void toggled(bool value);

    void loadActionPressed(void);

    void lastFrameToggled(void);

    void actionClearTriggered(void);

    void tabChangedEvent(int);

    void keypointsToggled(void);

    void registrationFrameDoneSlot(int);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> meshViewer;
    PointCloudT::Ptr kinectCloud;
    PointCloudT::Ptr registratedCloud;
    PointCloudAT::Ptr key_cloud;
    std::vector<PointCloudT::Ptr> clouds;
    QTimer *tmrTimer;
    QMovie *movie;


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

};

#endif // PCLVIEWER_H
