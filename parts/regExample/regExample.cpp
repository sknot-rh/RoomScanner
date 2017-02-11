#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace pcl::search;
PointCloud<PointXYZ>::Ptr src, tgt;

float key = 1.0;
float normal = 0.5;
float fpfh = 1.0;
float corr = 1.0;


////////////////////////////////////////////////////////////////////////////////
void
estimateKeypoints (const PointCloud<PointXYZ>::Ptr &src, 
                   const PointCloud<PointXYZ>::Ptr &tgt,
                   PointCloud<PointXYZ> &keypoints_src,
                   PointCloud<PointXYZ> &keypoints_tgt)
{
  // Get an uniform grid of keypoints
  UniformSampling<PointXYZ> uniform;
  uniform.setRadiusSearch (key);  // 1m

  uniform.setInputCloud (src);
  uniform.filter (keypoints_src);

  uniform.setInputCloud (tgt);
  uniform.filter (keypoints_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer source_pcd keypoints_src.pcd -ps 1 -ps 10
  savePCDFileBinary ("keypoints_src.pcd", keypoints_src);
  savePCDFileBinary ("keypoints_tgt.pcd", keypoints_tgt);
}

////////////////////////////////////////////////////////////////////////////////
void
estimateNormals (const PointCloud<PointXYZ>::Ptr &src, 
                 const PointCloud<PointXYZ>::Ptr &tgt,
                 PointCloud<Normal> &normals_src,
                 PointCloud<Normal> &normals_tgt)
{
  NormalEstimation<PointXYZ, Normal> normal_est;
  
  print_info ("computation for %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());
  normal_est.setInputCloud (src);
  normal_est.setRadiusSearch (normal);  // 50cm
  normal_est.compute (normals_src);

  normal_est.setInputCloud (tgt);
  normal_est.compute (normals_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer normals_src.pcd
  /*PointCloud<PointNormal> s, t;
  copyPointCloud<PointXYZ, PointNormal> (*src, s);
  copyPointCloud<Normal, PointNormal> (normals_src, s);
  copyPointCloud<PointXYZ, PointNormal> (*tgt, t);
  copyPointCloud<Normal, PointNormal> (normals_tgt, t);
  savePCDFileBinary ("normals_src.pcd", s);
  savePCDFileBinary ("normals_tgt.pcd", t);*/
}

////////////////////////////////////////////////////////////////////////////////
void
estimateFPFH (const PointCloud<PointXYZ>::Ptr &src, 
              const PointCloud<PointXYZ>::Ptr &tgt,
              const PointCloud<Normal>::Ptr &normals_src,
              const PointCloud<Normal>::Ptr &normals_tgt,
              const PointCloud<PointXYZ>::Ptr &keypoints_src,
              const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
              PointCloud<FPFHSignature33> &fpfhs_src,
              PointCloud<FPFHSignature33> &fpfhs_tgt)
{
  FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh_est;
  
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
  fpfh_est.setSearchMethod (tree);
  fpfh_est.setInputCloud (keypoints_src);
  fpfh_est.setInputNormals (normals_src);
  fpfh_est.setRadiusSearch (fpfh); // 1m
  fpfh_est.setSearchSurface (src);
  fpfh_est.compute (fpfhs_src);

  fpfh_est.setInputCloud (keypoints_tgt);
  fpfh_est.setInputNormals (normals_tgt);
  fpfh_est.setSearchSurface (tgt);
  fpfh_est.compute (fpfhs_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer fpfhs_src.pcd
  /*PCLPointCloud2 s, t, out;
  toPCLPointCloud2 (*keypoints_src, s); toPCLPointCloud2 (fpfhs_src, t); concatenateFields (s, t, out);
  savePCDFile ("fpfhs_src.pcd", out);
  toPCLPointCloud2 (*keypoints_tgt, s); toPCLPointCloud2 (fpfhs_tgt, t); concatenateFields (s, t, out);
  savePCDFile ("fpfhs_tgt.pcd", out);*/
}

////////////////////////////////////////////////////////////////////////////////
void
findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
                     const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
                     Correspondences &all_correspondences)
{
  CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                          const PointCloud<PointXYZ>::Ptr &keypoints_src,
                          const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                          Correspondences &remaining_correspondences)
{
  CorrespondenceRejectorDistance rej;
  rej.setInputSource<PointXYZ> (keypoints_src);
  rej.setInputTarget<PointXYZ> (keypoints_tgt);
  rej.setMaximumDistance (corr);    // 1m
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}


////////////////////////////////////////////////////////////////////////////////
void
computeTransformation (const PointCloud<PointXYZ>::Ptr &src, 
                       const PointCloud<PointXYZ>::Ptr &tgt,
                       Eigen::Matrix4f &transform)
{
  // Get an uniform grid of keypoints
  PointCloud<PointXYZ>::Ptr keypoints_src (new PointCloud<PointXYZ>), 
                            keypoints_tgt (new PointCloud<PointXYZ>);

  estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
  print_info ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

  // Compute normals for all points keypoint
  PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), 
                          normals_tgt (new PointCloud<Normal>);
  estimateNormals (src, tgt, *normals_src, *normals_tgt);
  print_info ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

  // Compute FPFH features at each keypoint
  PointCloud<FPFHSignature33>::Ptr fpfhs_src;
  fpfhs_src.reset (new PointCloud<FPFHSignature33>);
  PointCloud<FPFHSignature33>::Ptr fpfhs_tgt;
  fpfhs_tgt.reset (new PointCloud<FPFHSignature33>);
  estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

  // Copy the data and save it to disk
/*  PointCloud<PointNormal> s, t;
  copyPointCloud<PointXYZ, PointNormal> (*keypoints_src, s);
  copyPointCloud<Normal, PointNormal> (normals_src, s);
  copyPointCloud<PointXYZ, PointNormal> (*keypoints_tgt, t);
  copyPointCloud<Normal, PointNormal> (normals_tgt, t);*/

  // Find correspondences between keypoints in FPFH space
  CorrespondencesPtr all_correspondences (new Correspondences), 
                     good_correspondences (new Correspondences);
  findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

  // Reject correspondences based on their XYZ distance
  rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

  for (int i = 0; i < good_correspondences->size (); ++i)
    std::cerr << good_correspondences->at (i) << std::endl;
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input source PCD file and one input target PCD file to continue.\n");
    print_error ("Example: %s source.pcd target.pcd\n", argv[0]);
    return (-1);
  }

  // Load the files
  print_info ("Loading %s as source and %s as target...\n", argv[p_file_indices[0]], argv[p_file_indices[1]]);
  src.reset (new PointCloud<PointXYZ>);
  tgt.reset (new PointCloud<PointXYZ>);
  if (loadPCDFile (argv[p_file_indices[0]], *src) == -1 || loadPCDFile (argv[p_file_indices[1]], *tgt) == -1)
  {
    print_error ("Error reading the input files!\n");
    return (-1);
  }
  
  key = atof(argv[3]);
  normal = atof(argv[4]);
  fpfh = atof(argv[5]);
  corr = atof(argv[6]);
  
  printf("key %f\nnormal %f\nfpfh %f\ncorr %f\n\n",key, normal, fpfh, corr);
  
  print_info ("clouds have %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());
  
  PointCloud<PointXYZ>::Ptr srcf, tgtf;
  srcf.reset (new PointCloud<PointXYZ>);
  tgtf.reset (new PointCloud<PointXYZ>);
  VoxelGrid<PointXYZ> vg;
  vg.setInputCloud (src);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*srcf);
  
  vg.setInputCloud (tgt);
  vg.filter (*tgtf);

  src = srcf;
  tgt = tgtf;
  
  print_info ("after voxel grid clouds have %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());
  
  
  std::vector<int> indices;
  removeNaNFromPointCloud(*src,*src, indices);
  removeNaNFromPointCloud(*tgt,*tgt, indices);
  
  PointCloud<PointXYZ>::Ptr src_cloud_filtered;
  PointCloud<PointXYZ>::Ptr tgt_cloud_filtered;
  StatisticalOutlierRemoval<PointXYZ> sor;
  src_cloud_filtered.reset (new PointCloud<PointXYZ>);
  tgt_cloud_filtered.reset (new PointCloud<PointXYZ>);
  sor.setInputCloud (src);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*src_cloud_filtered);
  
  sor.setInputCloud (tgt);
  sor.filter (*tgt_cloud_filtered);
  
  src = src_cloud_filtered;
  tgt = tgt_cloud_filtered;
  
  
  print_info ("after filtering clouds have %lu and %lu points for the source and target datasets.\n", src->points.size (), tgt->points.size ());
  
  
  
  
  // Compute the best transformtion
  Eigen::Matrix4f transform;
  printf("filtrovani OK\n");
  computeTransformation (src, tgt, transform);

  std::cerr << transform << std::endl;
  // Transform the data and write it to disk
  PointCloud<PointXYZ> output;
  transformPointCloud (*src, output, transform);
  savePCDFileBinary ("source_transformed.pcd", output);
  printf("kokotina pojebana\n");
}
/* ]--- */
