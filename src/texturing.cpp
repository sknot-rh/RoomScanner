#include "texturing.h"

texturing::texturing()
{

}


bool texturing::stitchImages(std::vector<std::string> images) {
    std::vector<cv::Mat> imgs;
    std::string result_name = "texture.jpg";



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
         printf("Can't stitch images, error code = %s\n", status);
         return -1;
     }

    cv::imwrite(result_name, pano);

}
