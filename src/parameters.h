#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>

class parameters
{

private:
    static parameters* instance;
    parameters() {}
public:

    static parameters *GetInstance() {
        if (instance == NULL) {
            instance = new parameters();
        }
        return instance;
    }


    int param1 = 42;

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
};



#endif // PARAMETERS_H
