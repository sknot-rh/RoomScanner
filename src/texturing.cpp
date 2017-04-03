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
