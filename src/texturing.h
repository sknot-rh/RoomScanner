#ifndef TEXTURING_H
#define TEXTURING_H

#include "types.h"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"


class texturing
{
public:
    texturing();
    static bool stitchImages(std::vector<std::string> images);

};

#endif // TEXTURING_H
