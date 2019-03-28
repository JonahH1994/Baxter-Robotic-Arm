#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include "../include/occupancy_grid/occupancy_grid.h"
#include <unistd.h>
#include <vector>
//#include <sensor_msgs/PointCloud2>
//#include <pcl/point_cloud.h>
using namespace occupancy_grid ;
//OccupancyGrid grid;
octomap::OcTree* tree ;
//OccupancyGrid::OccupancyGrid grid(msg.resolution, 3, 3, 3, pmin) ;
void octomap_callback(const octomap_msgs::Octomap& msg){

	std::cout << msg.resolution << std::endl ;
	//std::cout <<  msg.data[0].resolution<< std::endl ;
	double tree_resolution = static_cast<double>(msg.resolution) ;
	tree = new octomap::OcTree(tree_resolution) ;
	int count = 0;
	tree =dynamic_cast<octomap::OcTree*>( octomap_msgs::binaryMsgToMap(msg) );
	octomap::OcTreeNode* rn = tree->getRoot() ;
	unsigned int max_depth = tree->getTreeDepth() ;
	std::cout << "Success..."<< std::endl ;
	//std::cout << tree->size() << std::endl ;
	std::cout << "Root value: " << rn->getOccupancy() << std::endl ;
	double minx, miny, minz, maxx, maxy, maxz = 0.0 ;
	//tree->getMetricSize(minx, miny, minz) ;
	tree->getMetricMax(maxx, maxy, maxz) ;
	tree->getMetricMin(minx, miny, minz) ;
	std::cout << "Min Size: x = " << minx << " y = " << miny << " z = " << minz << std::endl ;
	std::cout << "Max Size: x = " << maxx << " y = " << maxy << " z = " << maxz << std::endl ;
	/*
	for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it){
		// Fetching the coordinates in octomap-space
		if (tree->isNodeOccupied(*it) && it.getDepth() <= max_depth) {
		//	std::cout << "  x = " << it.getX() << std::endl;
			std::cout << "  y = " << it.getY() << std::endl;
			std::cout << "  z = " << it.getZ() << std::endl;
			std::cout << "  size = " << it.getSize() << std::endl;
			std::cout << "  depth = " << it.getDepth() << std::endl;
		///
			//count += 1 ;
			if (it.getX() < minx && it.getY() < miny && it.getZ() < minz) {
				minx = it.getX() ;
				miny = it.getY() ;
				minz = it.getZ() ;
			}
			//if (it.getX() < 0 || it.getY() < 0 || it.getZ() < 0) count += 1;
		}
		std::cout << "x = " << it.getX() << std::endl ;
		std::cout << "y = " << it.getY() << std::endl ;
		std::cout << "z = " << it.getZ() << std::endl ;
		count +=1 ;
		std::cout << "Size: " << it.getSize() << std::endl ;
		std::cout << "Depth: " << it.getDepth() << std::endl ;
	}*/
	std::cout << "size of depth: " << tree->getNodeSize(max_depth-2) << std::endl ;
	//std::cout << "Number of hits: " << count << std::endl ;
	//tree->getMetricMin(min_x, min_y, min_z) ;
	//std::cout << "loop min: x=" << minx << " y=" << miny << " z=" << minz << std::endl ;
	//std::cout << "Metric Min: x=" << min_x << " y=" << min_y << " z=" << min_z << std::endl ;
	//double maxx, maxy, maxz ;
	//tree->getMetricMax(maxx, maxy, maxz );
	//std::cout << "Metric Max: x=" << maxx << " y=" << maxy << " z=" << maxz << std::endl ;
	//size_t s = sizeof(msg.data)/sizeof(msg.data[0]) ;
	//std::cout << "Size of original array " << s << std::endl ;
	//int scale = 1/msg.resolution ;
	//double num = (maxx-min_x)*scale * (maxy-min_y)*scale * (maxz-min_z)*scale ;
	//std::cout << "Proposed occup grid size " << num << std::endl ;
}


// void p_callback(const sensor_msgs::PointCloud2& p2_cloud) {
// 	octomap::Octree* tree ;
// }

int main(int argc, char **argv){

	ros::init(argc, argv, "map_listener");
	ros::NodeHandle n ;
	ros::Rate r(10) ;
	ros::Subscriber sub = n.subscribe("/octomap_binary", 1000, octomap_callback);
	//ros::Subscriber sub = n.subscribe("/camera_ir/camera/depth/points", 1000, p_callback)
	ros::spin() ;
	
	usleep(10000) ;
	std::cout << "Moving on..." << std::endl ;
	while(ros::ok()) {
		if (tree){
			break ;
		}
		r.sleep() ;
	}

	
	double maxx, maxy, maxz, minx, miny, minz ;
	tree->getMetricMin(minx, miny, minz) ;
	tree->getMetricMax(maxx, maxy, maxz) ;
	//std::vector<double> pmin(3) ; //{minx, miny-tree->getResolution(), minz} ;
	minx -= tree->getResolution() ;
	miny -= tree->getResolution() ;
	minz -= tree->getResolution() ;
	//const std::vector<double> pmin{minx, miny, minz} ;
	//octomap::point3d* pmin = new octomap::point3d(float(minx), float(miny), float(minz)) ;
	//pmin[0] = minx ; pmin[1] = miny ; pmin[2] = minz ;
	//grid = new OccupancyGrid(tree->getResolution(), maxx-minx, maxy-miny, maxz-minz, octomap::point3d(float(minx), float(miny), float(minz)) ) ;
	//grid = new OccupancyGrid(tree->getResolution(), maxx-minx, maxy-miny, maxz-minz, minx, miny, minz ) ;
	OccupancyGrid grid(tree->getResolution(), maxx-minx, maxy-miny, maxz-minz, minx, miny, minz ) ;
	while(ros::ok()) {
		grid.update(tree) ;
		std::cout << "Occupancy grid updated..." << std::endl ;
		r.sleep() ;
	}
}

