#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <string> 

class SimpleOpenNIViewer
{
    public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

public: int counter = 0;
    
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
        std::string s= std::to_string(counter++);
        pcl::io::savePCDFile("test_pcd" + s +".pcd", *cloud);
        std::cout << "frame " <<  s <<  "\n" ;
        return;
        
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
    
};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
