#ifndef POINTREPR_H
#define POINTREPR_H

#include "types.h"
#include <pcl/point_representation.h>

// Define a new point representation for < x, y, z, curvature >
class PointRepr : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  PointRepr ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


#endif // POINTREPR_H
