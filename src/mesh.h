#ifndef MESH_H
#define MESH_H

#include "types.h"
#include "parameters.h"
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes.h>

class mesh
{
public:
    mesh();
    static pcl::PolygonMesh smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth); //todo bad allocation
    static void polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles); //bug in pcl??
};

#endif // MESH_H
