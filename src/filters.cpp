#include "filters.h"

filters::filters()
{

}


void filters::voxelGridFilter(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float leaf) {
    std::cout<<"downsampling filter " << leaf << "\n";

    parameters* params = parameters::GetInstance();
    std::ifstream config_file("config.json");

    if (!config_file.fail()) {
        std::cout << "Config file loaded\n";
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
            if (array_element.first == "gridFilter")
                std::cout << "gridFilter" << "\n";
            for (auto & property: array_element.second) {
                if (array_element.first == "gridFilter")
                    std::cout << " "<< property.first << " = " << property.second.get_value < std::string > () << "\n";
            }
        }

        params->VGFleafSize = pt.get<float>("gridFilter.leafSize");
    }


    pcl::VoxelGrid<PointT> ds;  //create downsampling filter
    //pcl::UniformSampling<PointT> ds;  //create downsampling filter
    ds.setInputCloud (cloudToFilter);
    if (leaf > 0.00000f) {
        printf("VoxelGrid parameter with priority %f\n", leaf);
        ds.setLeafSize (leaf, leaf, leaf);
    }
    else {
        ds.setLeafSize (params->VGFleafSize, params->VGFleafSize, params->VGFleafSize);
    }
    ds.filter (*filtered);
    std::cout<<"Filtered points: " << filtered->points.size() << "\n";
}

void filters::downsample (const PointCloudT::Ptr &input,  PointCloudT &output, double radius) {
    // Get an uniform grid of keypoints
    pcl::UniformSampling<PointT> uniform;
    uniform.setRadiusSearch (radius);

    uniform.setInputCloud (input);
    uniform.filter (output);
}


void filters::cloudSmooth(PointCloudT::Ptr cloudToSmooth, PointCloudT::Ptr output) {
    std::cout<<"smoothing "<< cloudToSmooth->points.size() <<" points\n";
    // final version will load from json while startup and changes will be done in GUI

    parameters* params = parameters::GetInstance();
    std::ifstream config_file("config.json");


    if (!config_file.fail()) {
        std::cout << "Config file loaded\n";
        using boost::property_tree::ptree;
        ptree pt;
        read_json(config_file, pt);

        for (auto & array_element: pt) {
            if (array_element.first == "mls")
                std::cout << "mls" << "\n";
            for (auto & property: array_element.second) {
                if (array_element.first == "mls")
                    std::cout << " "<< property.first << " = " << property.second.get_value < std::string > () << "\n";
            }
        }

        params->MLSpolynomialOrder = pt.get<int>("mls.polynomialOrder");
        params->MLSusePolynomialFit = pt.get<bool>("mls.usePolynomialFit");
        params->MLSsearchRadius = pt.get<double>("mls.searchRadius");
        params->MLSsqrGaussParam = pt.get<double>("mls.sqrGaussParam");
        params->MLSupsamplingRadius = pt.get<double>("mls.upsamplingRadius");
        params->MLSupsamplingStepSize = pt.get<double>("mls.upsamplingStepSize");
        params->MLSdilationIterations = pt.get<int>("mls.dilationIterations");
        params->MLSdilationVoxelSize = pt.get<double>("mls.dilationVoxelSize");
        params->MLScomputeNormals = pt.get<bool>("mls.computeNormals");
    }



    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud (cloudToSmooth);
    mls.setSearchRadius (params->MLSsearchRadius);
    mls.setSqrGaussParam (params->MLSsqrGaussParam);
    mls.setPolynomialFit (params->MLSusePolynomialFit);
    mls.setPolynomialOrder (params->MLSpolynomialOrder);

    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
    //  mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, pcl::PointXYZRGB>::NONE);
    mls.setPointDensity ( int (60000 * params->MLSsearchRadius)); // 300 points in a 5 cm radius
    mls.setUpsamplingRadius (params->MLSupsamplingRadius);
    mls.setUpsamplingStepSize (params->MLSupsamplingStepSize);
    mls.setDilationIterations (params->MLSdilationIterations);
    mls.setDilationVoxelSize (params->MLSdilationVoxelSize);

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    //pcl::search::OrganizedNeighbor<PointT> tree (new pcl::search::OrganizedNeighbor<PointT> ());
    mls.setSearchMethod (tree);
    mls.setComputeNormals (params->MLScomputeNormals);
    mls.process (*output);
    std::cout<<"now we have "<< output->points.size() <<" points\n";

}

void filters::oultlierRemoval(PointCloudT::Ptr cloudToFilter, PointCloudT::Ptr filtered, float radius) {
    /*pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloudToFilter);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered);*/

    pcl::RadiusOutlierRemoval<PointT> rorfilter;
    rorfilter.setInputCloud (cloudToFilter);
    rorfilter.setRadiusSearch (radius);
    rorfilter.setMinNeighborsInRadius (5);
    rorfilter.filter (*filtered);
}

