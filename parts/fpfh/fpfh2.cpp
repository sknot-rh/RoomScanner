#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <boost/filesystem.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <vector>

using namespace std;
using namespace boost;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZI KeyType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudPtr;
typedef Eigen::Matrix<float,4,4> Matrix4;

const float min_scale = 0.01;
const int nr_octaves = 3;
const int nr_scales = 3;
const float contrast = 10;

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));


    //viewer->addPointCloud<PointType> (b, last_rgb, "last cloud", vl1);
    //viewer->addCorrespondences<PointType>(kpts, last_kpts, *features_corrs, "cor", vl1);

            /*pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
            pcl::visualization::PointCloudColorHandlerRGBField<PointType> last_rgb(last_cloud);
            viewer->removePointCloud("cloud", vl1);
            viewer->addPointCloud<PointType> (cloud, rgb, "cloud", vl1);
            viewer->removePointCloud("last_cloud", vl1);
            viewer->addPointCloud<PointType> (last_cloud, last_rgb, "last_cloud", vl1);
            viewer->removeCorrespondences("cor", vl1);*/


int main(int argc, char** argv)
{

    int vl1(0), vl2(1);
    viewer->setBackgroundColor (255, 255, 255);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vl1);
    //Read in our files and sort them into the correct order
    boost::filesystem::path pcd_dir("/home/stanly/kinect/DP/RoomScanner/parts/fpfh/build/clouds");
    vector<boost::filesystem::path> pcd_files;
    vector<boost::filesystem::path>::iterator pcd_itr;
    copy(boost::filesystem::directory_iterator(pcd_dir),
            boost::filesystem::directory_iterator(), back_inserter(pcd_files));
    sort(pcd_files.begin(), pcd_files.end());


    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dense_cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dense_cloud2 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr kpts(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr kpts2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<NormalType>::Ptr normals2(new pcl::PointCloud<NormalType>());
    //Retain the last clouds
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
        features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
        features2(new pcl::PointCloud<pcl::FPFHSignature33>);
    //Load our files one by one and display their clouds

    
    //BOOST_FOREACH(boost::filesystem::path pcd_file, pcd_files)
    std::vector<int> indicies;

        boost::filesystem::path pcd_file = pcd_files[0];
        if(pcl::io::loadPCDFile<PointType>(pcd_file.string(), *dense_cloud) == -1)  
        {
            PCL_ERROR("Couldn't read file!");
            return(-1);
        }

        pcl::removeNaNFromPointCloud(*dense_cloud, *dense_cloud, indicies);

        vector<Eigen::Matrix<float, 4, 4> > transformations;

    for(int x=0; x<3; x++)
    {

        pcl::VoxelGrid<pcl::PointXYZRGBA> vox_grid;
        vox_grid.setLeafSize (0.01, 0.01, 0.01);
        vox_grid.setInputCloud (dense_cloud);
        vox_grid.filter (*cloud);


        //for(int y=x+1; y<pcd_files.size(); y++)
        //{
            boost::filesystem::path pcd_file2 = pcd_files[x+1];
            if(pcl::io::loadPCDFile<PointType>(pcd_file2.string(), *dense_cloud2) == -1)  
            {
                PCL_ERROR("Couldn't read file!");
                return(-1);
            }    

            pcl::removeNaNFromPointCloud(*dense_cloud2, *dense_cloud2, indicies);
            std::cout << "Loaded" << cloud->width * cloud->height << "data points " << std::endl;

            //Downsample


            //pcl::VoxelGrid<pcl::PointXYZRGBA> vox_grid;
            //vox_grid.setLeafSize (0.01, 0.01, 0.01);
            vox_grid.setInputCloud (dense_cloud2);
            vox_grid.filter (*cloud2);

       //Compute Keypoints
            pcl::PointCloud<KeyType>::Ptr keypoints (new pcl::PointCloud<KeyType>);
            pcl::SIFTKeypoint<PointType, KeyType> sift;
            sift.setScales(min_scale, nr_octaves, nr_scales);
            sift.setMinimumContrast(contrast);

            sift.setInputCloud(cloud);
            sift.setSearchSurface(cloud);
            sift.compute(*keypoints);
            kpts->points.resize(keypoints->points.size());
            pcl::copyPointCloud(*keypoints, *kpts);
            std::cout << "Found " << keypoints->points.size() << " keypoints." << std::endl;

            sift.setInputCloud(cloud2);
            sift.setSearchSurface(cloud2);
            sift.compute(*keypoints);
            kpts->points.resize(keypoints->points.size());
            pcl::copyPointCloud(*keypoints, *kpts2);
            std::cout << "Found " << keypoints->points.size() << " keypoints." << std::endl;

            //Descriptors
            //Compute Normals
            pcl::NormalEstimationOMP<PointType, NormalType> norm;
            norm.setKSearch(10);
            norm.setInputCloud(kpts);
            norm.compute(*normals);

            norm.setInputCloud(kpts2);
            norm.compute(*normals2);

            std::cout << "Computing descriptors..." << std::endl;
            pcl::FPFHEstimation<PointType, NormalType, pcl::FPFHSignature33> fpfh;
            fpfh.setRadiusSearch(0.08);

            fpfh.setInputCloud(kpts);
            fpfh.setInputNormals(normals);
            fpfh.compute(*features);

            fpfh.setInputCloud(kpts2);
            fpfh.setInputNormals(normals2);
            fpfh.compute(*features2);

            pcl::CorrespondencesPtr features_corrs (new pcl::Correspondences ());
            pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> c_est;
            c_est.setInputCloud(features);
            c_est.setInputTarget(features2);
            c_est.determineReciprocalCorrespondences(*features_corrs);
            //Calculate Correlations

            //RANSAC
            std::cout << "Found " << features_corrs->size() << " correspondences" <<
                " between frame x:" << x << " and y:" << x+1 << std::endl;

            //Features = P
            //Features2 = Q
            int num_iterations = 100;
            int num_samples = 5;
            Eigen::Matrix<float,4,4> Rt;
            Eigen::Matrix<float,4,4> BestRt;
            pcl::PointCloud<PointType>::Ptr cloud_tr(new pcl::PointCloud<PointType>);
            float best;

            //Filter to extract subsamples
            for(int i=0; i<num_iterations; i++)
            {
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
                    s(new pcl::PointCloud<pcl::FPFHSignature33>);

                pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
                    q(new pcl::PointCloud<pcl::FPFHSignature33>);

               //Select a subsample
                for(int j=0; j<num_samples; j++)
                {
                    srand(time(NULL));
                    int ind = rand() % features->size();
                    //Store the indicies of the sample
                    s->points.push_back(features->points[ind]);
                }
                
                c_est.setInputCloud(s);
                c_est.setInputTarget(features2);
                c_est.determineReciprocalCorrespondences(*features_corrs);

                //Determine transformation error using points
                pcl::registration::TransformationEstimationSVD<PointType, PointType> svd;
                svd.estimateRigidTransformation(*kpts, *kpts2, *features_corrs, Rt);
                //Define some error function
                pcl::transformPointCloud(*cloud, *cloud_tr, Rt);
                float eps = 0;
                //Perform sum of squared differences
                for(int p1=0; p1 < cloud->points.size(); p1++)
                {
                    eps+= pow(cloud->points[p1].x - cloud_tr->points[p1].x, 2);
                    eps+= pow(cloud->points[p1].y - cloud_tr->points[p1].y, 2);
                    eps+= pow(cloud->points[p1].z - cloud_tr->points[p1].z, 2);
                }
                if(i==0)
                {
                    best = eps;
                    BestRt = Rt;
                    std::cout << "best:" << best;
                }
                else
                {
                    if(eps < best)
                    {
                        best = eps;
                        BestRt = Rt;
                    }
                }
                std::cout << best << std::endl;
            }
           
            transformations.push_back(BestRt);
            pcl::copyPointCloud(*cloud2, *cloud_tr);

            for(int t=transformations.size()-1; t > 0; t--)
                pcl::transformPointCloud(*cloud_tr, *cloud_tr, transformations[t]);
                   
            std::stringstream cs ("cloud");
            std::stringstream cs_tr ("cloud_tr");
            cs << x;
            cs_tr << x;
            pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(dense_cloud);
            viewer->addPointCloud<PointType> (dense_cloud, rgb, cs.str(), vl1);
            pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_tr(cloud_tr);
            viewer->addPointCloud<PointType> (cloud_tr, rgb_tr, cs_tr.str(), vl1);

            pcl::copyPointCloud(*cloud2, *cloud);

            //viewer->addCorrespondences<PointType>(kpts, kpts2, *features_corrs, "cor", vl1);
        //}
        
    }
    
    viewer->spin();
    return(0);
}
