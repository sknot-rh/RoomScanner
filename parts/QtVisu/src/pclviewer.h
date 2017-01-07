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

  void saveButtonPressed(void);

  void polyButtonPressed(void);

  void pSliderValueChanged(int value);

  void closing(void);

  void drawFrame(void);

  void toggled(bool value);

  void loadActionPressed(void);

  void lastFrameToggled(void);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> meshViewer;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr key_cloud;
  std::list<PointCloudT::Ptr> clouds;

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
  boost::mutex mtx_;

  // Parameters for sift computation
  const float min_scale = 0.1f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 10;
  const float min_contrast = 0.5f;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
