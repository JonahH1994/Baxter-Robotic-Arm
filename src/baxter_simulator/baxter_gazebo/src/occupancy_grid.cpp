//#include <stdio.h>
//#include "baxter_gazebo/include/occupancy_grid/occupancy_grid.h"
//#include "../include/occupancy_grid/occupancy_grid.h"
#include "occupancy_grid/occupancy_grid.h"
//#include <vector>
//#include "octomap/octomap_types.h"
//#include "octomap/math/Vector3.h"

namespace occupancy_grid
{
int occupied_val = 1 ;
int default_val = -1 ;

OccupancyGrid::OccupancyGrid(double resolution, double size, const Eigen::Vector3d pmin)
//OccupancyGrid::OccupancyGrid(double resolution, double size, const std::vector<double>& pmin)
//OccupancyGrid::OccupancyGrid(double resolution, double size, const octomap::point3d& pmin)
//OccupancyGrid::OccupancyGrid(double resolution, double size, double minx, double miny, double minz)
{
  OccupancyGrid(resolution, size, size, size, pmin);
  //OccupancyGrid(resolution, size, size, size, minx, miny, minz) ;
}

OccupancyGrid::OccupancyGrid(double resolution, double length, double width, double height, const Eigen::Vector3d pmin) :
//OccupancyGrid::OccupancyGrid(double resolution, double length, double width, double height, const std::vector<double>& pmin) :
//OccupancyGrid::OccupancyGrid(double resolution, double length, double width, double height, const octomap::point3d& pmin ) :
//OccupancyGrid::OccupancyGrid(double resolution, double length, double width, double height, double minx, double miny, double minz ) :
  resolution_(resolution)
  ,op_min_(pmin)
{
  scale_ = floor(1/resolution) ;
  length_ = int(length * scale_ );
  width_ = int(width * scale_ );
  height_ = int(height * scale_ ); 
  int size = length_ * width_ * height_ ;
  occupancy_grid = new int[size];
  //op_min_[0] = minx ; //pmin.x() ;
  //op_min_[1] = miny ; //pmin.y() ;
  //op_min_[2] = minz ; //pmin.z() ;
  for (int i = 0; i < size; i++) {
    occupancy_grid[i] = default_val ;
  }
}

OccupancyGrid::~OccupancyGrid()
{
  delete[] occupancy_grid;
}

OccupancyGrid::OccupancyGrid() : resolution_(0), length_(0), width_(0), height_(0), scale_(0) {}


//int OccupancyGrid::posToIndex(const std::vector<int> grid_pos)
int OccupancyGrid::posToIndex(const Eigen::Vector3i grid_pos)
{
  assert((grid_pos(0) >= 0) && (grid_pos(1) >= 0) && (grid_pos(2) >= 0)) ;
  int indx = grid_pos(0) + length_*grid_pos(1) + length_*width_*grid_pos(2) ;
  return indx ;
}

//std::vector<int> OccupancyGrid::octPosToGrid(const std::vector<double> octo_pos)
Eigen::Vector3i OccupancyGrid::octPosToGrid(const Eigen::Vector3d octo_pos)
{
  //std::vector<int> new_pos(3) ; // = new std::vector<int>(3);
  Eigen::Vector3i new_pos ;
  new_pos(0) = scale_ * (int)floor(octo_pos(0) - op_min_(0)) ;
  new_pos(1) = scale_ * (int)floor(octo_pos(1) - op_min_(1)) ;
  new_pos(2) = scale_ * (int)floor(octo_pos(2) - op_min_(2)) ;

  return new_pos;
}

//std::vector<double> OccupancyGrid::gridPosToOcto(const std::vector<int> grid_pos)
Eigen::Vector3d OccupancyGrid::gridPosToOcto(const Eigen::Vector3i grid_pos)
{
  //std::vector<double> new_pos(3) ; // = new std::vector<double>(3) ;
  Eigen::Vector3d new_pos ; // = new std::vector<double>(3) ;
  new_pos(0) = double( grid_pos(0)/scale_ ) + op_min_(0) ;
  new_pos(1) = double( grid_pos(1)/scale_ ) + op_min_(1) ;
  new_pos(2) = double( grid_pos(2)/scale_ ) + op_min_(2) ;
  // this may be a little more complicated since some of the values are odd indices 
  return new_pos ;
}

//bool OccupancyGrid::isOccupied(const std::vector<int> grid_pos)
bool OccupancyGrid::isOccupied(const Eigen::Vector3i grid_pos)
{
  int pos = posToIndex(grid_pos);
  return occupancy_grid[pos] == occupied_val;
}

//bool OccupancyGrid::isOccupied(const std::vector<double> octo_pos)
bool OccupancyGrid::isOccupied(const Eigen::Vector3d octo_pos)
{
  Eigen::Vector3i grid_pos = octPosToGrid(octo_pos) ;
  //std::vector<int> grid_pos = octPosToGrid(octo_pos) ;
  return isOccupied(grid_pos) ;
}

//void OccupancyGrid::setOccupied(const std::vector<int> grid_pos, int set_val )
void OccupancyGrid::setOccupied(const Eigen::Vector3i grid_pos, int set_val )
{
  int pos = posToIndex(grid_pos) ;
  occupancy_grid[pos] = set_val ;
}

//void OccupancyGrid::update_recurse(const std::vector<double> center, double depth, int value) {
void OccupancyGrid::update_recurse(const Eigen::Vector3d center, double depth, int value) {
  if (depth == resolution_) {
    // Base Case: only one cell
    setOccupied(octPosToGrid(center), value) ;
  } else {
    // Divide the current center into 8 sub cubes
    for (int i = 0; i < 8; i++) {
      std::cout << "value of center: " << center << " value of offset: " << offsets[i] << std::endl ;
      std::cout << "Value of depth: " << depth << std::endl ;
      update_recurse(center+offsets[i]*(depth/4), depth/2, value) ;
    }
  }
}

void OccupancyGrid::update(octomap::OcTree* tree) {
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; it++) {
    octomap::point3d cord = it.getCoordinate() ;
    //std::vector<double> curr_opd(3) ; // = new std::vector<double>(3) ;
    Eigen::Vector3d curr_opd ;
    curr_opd(0) = cord.x() ;
    curr_opd(1) = cord.y() ;
    curr_opd(2) = cord.z() ;

    //vector<int> curr_opi = octPosToGrid(curr_opd) ;
    int value = (tree->isNodeOccupied(*it)) ? 1 : 0 ;
    update_recurse(curr_opd, it.getDepth(), value) ;
    //setOccupied(curr_opi, value) ;
  }
}

}
