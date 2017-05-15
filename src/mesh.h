/*
    This file is part of RoomScanner.

    RoomScanner is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RoomScanner is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with RoomScanner.  If not, see <http://www.gnu.org/licenses/>.
*/

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
#include <pcl/surface/grid_projection.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


class mesh
{

public:
    mesh();
    static void smoothMesh(pcl::PolygonMesh::Ptr meshToSmooth, pcl::PolygonMesh::Ptr output); //todo bad allocation
    static void polygonateCloudGreedyProj(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void polygonateCloudMC(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles); //bug in pcl
    static void fillHoles(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut);
    static void polygonateCloudPoisson(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void meshDecimation(pcl::PolygonMesh::Ptr trianglesIn, pcl::PolygonMesh::Ptr trianglesOut);
    static void polygonateCloudGridProj(PointCloudT::Ptr cloudToPolygonate, pcl::PolygonMesh::Ptr triangles);
    static void retextureMesh(PointCloudT::Ptr originCloud, pcl::PolygonMesh::Ptr triangles);

};

#endif // MESH_H
