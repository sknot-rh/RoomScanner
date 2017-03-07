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
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkFillHolesFilter.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

class mesh
{

public:
    mesh();
    static pcl::PolygonMesh smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth); //todo bad allocation
    static void polygonateCloud(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles); //bug in pcl
    static void fillHoles(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut);
    static void polygonateCloudPoisson(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void meshDecimation(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut);

};

#endif // MESH_H
