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

#include "texturing.h"

texturing::texturing()
{

}

/** \brief Stitch N images
  * \param images names of images which will be stitched
  * \return true if stitching was successfull
  */
bool texturing::stitchImages(std::vector<std::string> images) {
    std::vector<cv::Mat> imgs;
    std::string result_name = "texture.jpg";

    if (images.size() < 2) {
        return false;
    }

    for (int i = 0; i < images.size(); ++i)
    {

     {
         cv::Mat img = cv::imread(images[i].c_str());
         if (img.empty())
         {
             printf("Can't read image ' %s '\n", images[i].c_str());
             return false;
         }
         imgs.push_back(img);
     }
    }


     cv::Mat pano;
     cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::SCANS, false);
     cv::Stitcher::Status status = stitcher->stitch(imgs, pano);

     if (status != cv::Stitcher::OK)
     {
         printf("Can't stitch images, error code = %d\n", status);
         return false;
     }

    cv::imwrite(result_name, pano);
    printf("Creating texture done.\n");

}
