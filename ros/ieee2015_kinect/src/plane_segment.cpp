#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

class SimpleOpenNIProcessor
{
  public:
    SimpleOpenNIProcessor () : viewer ("PCL OpenNI Viewer") {}

    void rotate_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
    {

      float theta = M_PI/4; // The angle of rotation in radians
      Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
      // Define a translation of 2.5 meters on the x axis.
      const float kinect_height = 2.5;
      transform_2.translation() << 0.0, 0.0, kinect_height;
      transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
      pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);
      viewer.showCloud(transformed_cloud);

    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
     	if(!viewer.wasStopped())
      {
        SimpleOpenNIProcessor::rotate_cloud(cloud);
     		// viewer.showCloud (cloud);
        std::cout << "Banana" << std::endl;
     	}
    }

    void run ()
    {
      // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber();
      // make callback function from member function
      boost::function<void (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);
      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);
      // start receiving point clouds
      interface->start ();
      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (!viewer.wasStopped()) {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
  	  }
      // stop the grabber
      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}