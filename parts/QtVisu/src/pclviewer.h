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

typedef pcl::PointXYZRGBA PointAT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointAT> PointCloudAT;
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
  void cloud_cb_ (const PointCloudAT::ConstPtr &ncloud);
  void cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
  void voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered);
  void polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
  pcl::PolygonMesh smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth);
  void polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);

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
  PointCloudAT::Ptr cloud;
  PointCloudAT::Ptr key_cloud;
  std::list<PointCloudAT::Ptr> clouds;

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
  bool sensorConnected;

  // Parameters for sift computation
  const float min_scale = 0.1f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 10;
  const float min_contrast = 0.5f;

  // Parameters for MLS
  int MLSpolynomialOrder = 2;
  bool MLSusePolynomialFit = true;
  double MLSsearchRadius = 0.05;
  double MLSsqrGaussParam = 0.0025;
  double MLSupsamplingRadius = 0.025;
  double MLSupsamplingStepSize = 0.015;
  int MLSdilationIterations = 2;
  double MLSdilationVoxelSize = 0.01;
  bool MLScomputeNormals = false;

  // Parameters for Voxel Grid
  double VGFleafSize = 0.02;

  // Parameters for Greedy Projection
  double GPsearchRadius = 0.06;
  double GPmu = 2.5;
  int GPmaximumNearestNeighbors = 100;


private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
