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

boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  	viewer->addCoordinateSystem (1.0);
  	viewer->initCameraParameters ();
  	return (viewer);
}

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file to generate the Point CLoud.");
		return(-1);
	}else{
		cout << "Point Cloud Accepted with " << cloud->points.size() << " points.\n";
	}

	
	ros::init (argc, argv, "pub_pcl");
  	ros::NodeHandle nh;
  	ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  	cloud->header.frame_id = "map";

  	tf::TransformBroadcaster br;
  	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  	transform.setRotation(tf::Quaternion(0, 0, 0, 1));

	ros::Rate loop_rate(4);

	while (nh.ok())
	{
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "office_frame"));
	  pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
	  pub.publish (cloud);
	  cout << cloud->points.size() << endl;
	  ros::spinOnce ();
	  loop_rate.sleep ();
	}


	// boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
	// pcl_viewer = getViewer(cloud);

	// while(!pcl_viewer->wasStopped()){
	// 	pcl_viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }

}