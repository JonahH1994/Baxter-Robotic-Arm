#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdlib.h> 
#include <vector>
#include <Eigen/Dense>
//#include <Eigen/Core> 
#include <octomap/octomap.h>

namespace occupancy_grid
{

template <class T>
inline std::vector<T> operator +(const std::vector<T>& a, const std::vector<T>& b) {
	//assert(a.size() == b.size()) ;
	std::vector<T> c(a.size()) ;
	for (int i = 0; i < a.size(); i++) {
		c[i] = a[i] + b[i] ;
	}
}

template <class T>
inline std::vector<T> operator *(const std::vector<T>& a, const T& b) {
	std::vector<T> c(a.size()) ;
	for (int i = 0; i < a.size(); i++) {
		c[i] = a[i] * b ;
	}
}

class OccupancyGrid
{
        public:
                OccupancyGrid(double resolution, double size, const Eigen::Vector3d  pmin);
                //OccupancyGrid(double resolution, double size, const std::vector<double>&  pmin);
                //OccupancyGrid(double resolution, double size, const octomap::point3d& pmin);
		OccupancyGrid(double resolution, double length, double width, double height, const Eigen::Vector3d pmin);
		//OccupancyGrid(double resolution, double length, double width, double height, const std::vector<double>& pmin);
		//OccupancyGrid(double resolution, double length, double width, double height, const octomap::point3d& pmin);
                //OccupancyGrid(double resolution, double length, double width, double height, double minx, double miny, double minz);
		OccupancyGrid() ;
                ~OccupancyGrid();
                double getLength() {return length_;} 
               	double getWidth() {return width_; }
                double getHeight() {return height_; }
		double getScale() {return scale_; }
		double getSize() {return length_ * width_ * height_; }
		double getResolution() {return resolution_ ; }
                int * getOccupancyGrid() {return occupancy_grid; }
		Eigen::Vector3d getPMin() {return op_min_; }
		//std::vector<double> getPMin() {return op_min_; }
		//octomap::point3d getPMin() {return op_min_; }
		//int getResolution() { return resolution_ ; }
                bool isOccupied(const Eigen::Vector3d octo_pos );
                //bool isOccupied(const std::vector<double> op_pos ) ;
		bool isOccupied(const Eigen::Vector3i grid_pos ) ;
		//bool isOccupied(const std::vector<int> grid_pos ) ;
                void setOccupied(const Eigen::Vector3i grid_pos, int& set_val);
                //void setOccupied(const std::vector<int> grid_pos, int set_val);
                int posToIndex(const Eigen::Vector3i grid_pos);
                //int posToIndex(const std::vector<int> grid_pos);
               	Eigen::Vector3i octPosToGrid(const Eigen::Vector3d octo_pos) ;
                //std::vector<int> octPosToGrid(const std::vector<double> octo_pos) ;
                Eigen::Vector3d gridPosToOcto(const Eigen::Vector3i grid_pos) ;
                //std::vector<double> gridPosToOcto(const std::vector<int> grid_pos) ;
		void update(octomap::OcTree* tree) ;
		inline void operator =(OccupancyGrid* other){
			length_ = other->getLength() ;
			width_ = other->getWidth() ;
			height_ = other->getHeight() ;
			occupancy_grid = other->getOccupancyGrid() ;
			resolution_ = other->getResolution() ;
			op_min_ = other->getPMin() ;
		}
        private:
                int * occupancy_grid ;
                int length_, width_, height_, scale_ ;
                double resolution_ ;
		//std::vector<double> op_min_{0.0,0.0,0.0} ;
		Eigen::Vector3d op_min_ ;
		//void update_recurse(const std::vector<double> center, double depth, int value) ;
		void update_recurse(const Eigen::Vector3d center, double depth, int value) ;
		//std::vector<std::vector<double>> offsets{{1,1,1}, {1,-1,1}, {-1,-1,1},
		//	{-1,1,1}, {1,1,-1}, {1,-1,-1}, {-1,-1,-1}, {-1,1,-1}} ;
		const std::vector<Eigen::Vector3d> offsets{Eigen::Vector3d(1,1,1), Eigen::Vector3d(1,-1,1), Eigen::Vector3d(-1,-1,1),
			Eigen::Vector3d(-1,1,1), Eigen::Vector3d(1,1,-1), Eigen::Vector3d(1,-1,-1), Eigen::Vector3d(-1,-1,-1), Eigen::Vector3d(-1,1,-1)} ;
};
}

#endif
