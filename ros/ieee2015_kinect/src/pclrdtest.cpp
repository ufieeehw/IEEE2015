#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Test-pcd.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't_read_file_test_pcd_.pcd_\n");
		return(-1);
	}	
	return(0);
}