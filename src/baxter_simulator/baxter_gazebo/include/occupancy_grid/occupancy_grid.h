#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdlib.h> 
#include <vector> 
#include <octomap/octomap.h>
namespace occupancy_grid
{

template <class T>
inline std::vector<T> operator +(const std::vector<T>& a, const std::vector<T>& b) {
	assert(a.size() == b.size()) ;
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
                //OccupancyGrid(double resolution, double size, const octomap::point3d& pmin);
                OccupancyGrid(double resolution, double size, double minx, double miny, double minz);
                //OccupancyGrid(double resolution, double length, double width, double height, const std::vector<double>& pmin);
						
                //OccupancyGrid(double resolution, int size, octomap::point3d pmin);
                //OccupancyGrid(double resolution, double length, double width, double height, const octomap::point3d& pmin);
                OccupancyGrid(double resolution, double length, double width, double height, double minx, double miny, double minz);
		OccupancyGrid() ;
                ~OccupancyGrid();
                double getLength() {return length_;} 
               	double getWidth() {return width_; }
                double getHeight() {return height_; }
		double getScale() {return scale_; }
                int * getOccupancyGrid() {return occupancy_grid; }
		std::vector<double> getPMin() {return op_min_; }
		int getResolution() { return resolution_ ; }
                bool isOccupied(const std::vector<double> octo_pos );
                bool isOccupied(const std::vector<int> grid_pos ) ;
                void setOccupied(const std::vector<int> grid_pos, int set_val);
                int posToIndex(const std::vector<int> grid_pos);
                std::vector<int> octPosToGrid(const std::vector<double> octo_pos) ;
                std::vector<double> gridPosToOcto(const std::vector<int> grid_pos) ;
		void update(octomap::OcTree* tree) ;
		inline void operator =(OccupancyGrid* other){
			length_ = other->getLength() ;
			width_ = other->getWidth() ;
			height_ = other->getHeight() ;
			occupancy_grid = other->getOccupancyGrid() ;
			resolution_ = other->getResolution() ;
		} 
        private:
                int * occupancy_grid ;
                double length_, width_, height_, scale_ ;
                double resolution_ ;
		std::vector<double> op_min_{0.0,0.0,0.0} ;
		void update_recurse(const std::vector<double> center, double depth, int value) ;
		std::vector<std::vector<double>> offsets{{1,1,1}, {1,-1,1}, {-1,-1,1},
			{-1,1,1}, {1,1,-1}, {1,-1,-1}, {-1,-1,-1}, {-1,1,-1}} ;
};
}

#endif
