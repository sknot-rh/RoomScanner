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

    // Default parameters (config file is not found)

    // Parameters for sift computation
    double SIFTmin_scale = 0.003;
    int SIFTn_octaves = 8;
    int SIFTn_scales_per_octave = 10;
    double SIFTmin_contrast = 0.3;

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
    double REGfpfh = 0.25;
    double REGreject = 1.0;
    //icp
    double REGcorrDist = 0.2;

    // Parameters for Fast Bilateral Filter
    double FBFsigmaS = 10;
    double FBFsigmaR = 0.1;

    // Parameter for mesh decimation
    double DECtargetReductionFactor = 0.2; // 20%

    // Parameter for hole filling
    double HOLsize = 0;

};



#endif // PARAMETERS_H
