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
