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

typedef pcl::PointXYZRGBA PointAT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointAT> PointCloudAT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT
  QThread thread;

public:
  PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();
  void cloud_cb_ (const PointCloudAT::ConstPtr &ncloud);
  void cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output);
  void voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered);
  void polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
  pcl::PolygonMesh smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth);
  void polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
  void polyButtonPressedFunc();
  void loading(QLabel* label);
  PointCloudT::Ptr registrateNClouds();
  void pairAlign (const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
  void estimateKeypoints (const PointCloudT::Ptr &src,
                     const PointCloudT::Ptr &tgt,
                     PointCloudT &keypoints_src,
                     PointCloudT &keypoints_tgt);
  void estimateNormals (const PointCloudT::Ptr &src,
                   const PointCloudT::Ptr &tgt,
                   pcl::PointCloud<pcl::Normal> &normals_src,
                   pcl::PointCloud<pcl::Normal> &normals_tgt);
  void estimateFPFH (const PointCloudT::Ptr &src,
                const PointCloudT::Ptr &tgt,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals_tgt,
                const PointCloudT::Ptr &keypoints_src,
                const PointCloudT::Ptr &keypoints_tgt,
                pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src,
                pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_tgt);
  void findCorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                       const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                       pcl::Correspondences &all_correspondences);
  void rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                            const PointCloudT::Ptr &keypoints_src,
                            const PointCloudT::Ptr &keypoints_tgt,
                            pcl::Correspondences &remaining_correspondences);

  void computeTransformation (const PointCloudT::Ptr &src,
                         const PointCloudT::Ptr &tgt,
                         Eigen::Matrix4f &transform);
  void downsample (const PointCloudT::Ptr &src_origin, const PointCloudT::Ptr &tgt_origin, PointCloudT &src, PointCloudT &tgt);

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
  QLabel* labelRegistrate;
  QLabel* labelPolygonate;

};

#endif // PCLVIEWER_H
