#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();
  void cloud_cb_ (const PointCloudT::ConstPtr &ncloud);

public slots:
  void resetButtonPressed(void);

  void pSliderValueChanged(int value);

  void closing(void);

  void drawFrame(void);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;
  pcl::Grabber* interface;
  bool stream;
  bool copying;
  bool stop;
  int cloudWidth;
  int cloudHeight;
  std::vector<float> cloudX, cloudY, cloudZ;
  std::vector<unsigned long> cloudRGB;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
