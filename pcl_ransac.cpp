#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  	viewer->addCoordinateSystem (1.0);
  	viewer->initCameraParameters ();
  	return (viewer);
}

void callback(const PointCloud::ConstPtr& msg){
	printf ("Cloud: width = %d, height = %d", msg->width, msg->height);
	cout << ", frame_id = " << msg->header.frame_id << endl;
	cloud->points = msg->points;
	cloud->header.frame_id = msg->header.frame_id;
	cloud->width = msg->width;
	cloud->height = msg->height;
}


void visualize(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
	pcl_viewer = getViewer(cloud);

	while(!pcl_viewer->wasStopped()){
		pcl_viewer->spinOnce(100);
		pcl_viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main(int argc, char** argv){

	// if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
	// 	PCL_ERROR("Couldn't read file to generate the Point CLoud.");
	// 	return(-1);
	// }else{
	// 	cout << "Point Cloud Accepted with " << cloud->points.size() << " points.\n";
	// }

	
	ros::init (argc, argv, "sub_pcl");
	cout << "Debug1\n";
  	ros::NodeHandle nh;
  	cout << "Debug2\n";
  	ros::Subscriber sub = nh.subscribe<PointCloud> ("points2", 1, callback);
  	ros::AsyncSpinner spinner(1);
	spinner.start();
	// ros::WaitForShutdown();

	// while (nh.ok())
	// {
	//   pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
	//   pub.publish (cloud);
	//   cout << cloud->points.size() << endl;
	//   ros::spinOnce ();
	//   loop_rate.sleep ();
	// }

	visualize();


}