#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>

class parameters
{

private:
    static parameters* instance;
    parameters() {}
public:
    // singleton class
    static parameters *GetInstance() {
        if (instance == NULL) {
            instance = new parameters();
        }
        return instance;
    }

    Eigen::Quaternionf m;

    // Default parameters (config file is not found)

    // Parameters for sift computation
    double SIFTmin_scale = 0.05;
    int SIFTn_octaves = 8;
    int SIFTn_scales_per_octave = 10;
    double SIFTmin_contrast = 1.5;

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

    // Parameters for registration module
    //fpfh
    double REGnormalsRadius = 0.05;
    double REGfpfh = 1.5;
    double REGreject = 1.5;
    //icp
    double REGcorrDist = 0.2;

    // Parameters for Fast Bilateral Filter
    double FBFsigmaS = 10;
    double FBFsigmaR = 0.1;

    // Parameter for mesh decimation
    double DECtargetReductionFactor = 0.2; // 20%

    // Parameter for hole filling
    double HOLsize = 0.2;

    // Parameter for Grid projection

    double GRres = 0.01;

};



#endif // PARAMETERS_H
