#ifndef D_STAR_H
#define D_STAR_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <occupancy_grid/occupancy_grid.h>
#include <d_star/waypoint.h>
#include <math.h>
#include <unordered_map>
#include <queue>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace waypoint ;
using namespace occupancy_grid ;
namespace d_star
{

const double HIGH_COST = 10000 ;
class LessThanByKey{
	public:
		//bool operator() (const Key& k1, const Key& k2) const {
		bool operator() (const WayPoint& wp1, const WayPoint& wp2 ) const {
			return wp1.getKey() < wp2.getKey() ;
	}
};

class DStar
{

	public:
		DStar(OccupancyGrid* ogrid, Eigen::Vector3i& starting_point, Eigen::Vector3i& ending_point) ;
		bool isRaise(WayPoint point) ;
		double costVia(WayPoint current_point, WayPoint next_point) ;		
		double G(Eigen::Vector3i& point) ;
		double heuristic(Eigen::Vector3i& p1, Eigen::Vector3i& p2) ;	
		double heuristic(WayPoint wp1, WayPoint wp2) ;	
		double rhs(Eigen::Vector3i& p1, Eigen::Vector3i& p2) ;
		Key calculateKey(Eigen::Vector3i& point) ;
		Key calculateKey(WayPoint& wp) { return calculateKey( wp.getPoint() ) ; }
		void initialize() ;
		void updateVertex(WayPoint& wp) ;
		void updateVertex(WayPoint& wp1, WayPoint& wp2) ;
		void update(WayPoint& wp) ;
		void update(WayPoint& wp, int ins) ;
		void newCell(WayPoint& wp) ;
		void insert(WayPoint& wp) ;
		void insert(WayPoint& wp, int ins) ;
		void remove(WayPoint& wp) ;
		void computeShortestPath() ;	
		WayPoint top() ;
		WayPoint getPoint(Eigen::Vector3i& point, WayPoint& u) ;
		WayPoint getPoint(Eigen::Vector3i& point) ;
		double rhs(WayPoint wp) ;
		void Main() ;
		bool obstacleDetected(WayPoint& wp) ;
		void sendPath() ;
		Eigen::Vector3d gridToOct( Eigen::Vector3i point ) ;
		Eigen::Vector3d gridToOct( WayPoint wp ) ;
		int hashKey( Eigen::Vector3i point ) { return ogrid_->posToIndex( point ) ; }
		int hashKey( WayPoint wp ) { return hashKey( wp.getPoint() ) ; }

	private:
	
		OccupancyGrid* ogrid_ ;
		Eigen::Vector3i sp, gp ; // starting point and ending point	
		int length_, width_, height_, look_ahead_distance ;
		double km ;	
		std::unordered_map<int, WayPoint> table ; 
		std::priority_queue<WayPoint, std::vector<WayPoint>, LessThanByKey> Q ; // Open list	
		//std::priority_queue<WayPoint> Q ;
		WayPoint slast, start, sgoal ;
		Eigen::Vector3d oct_root_pos ;
		Eigen::Vector3d curr_pos ;
		bool reached_point, obstacle_detected ;	
		ros::NodeHandle p_nh ;
		ros::Publisher path_pub ;		

		std::vector<Eigen::Vector3i> new_directions{Eigen::Vector3i(-1,1,0), Eigen::Vector3i(0,1,0), Eigen::Vector3i(1,1,0), Eigen::Vector3i(-1,0,0), Eigen::Vector3i(0,0,0), Eigen::Vector3i(1,0,0), Eigen::Vector3i(-1,-1,0), Eigen::Vector3i(0,-1,0), Eigen::Vector3i(1,-1,0) } ;		

		std::vector<Eigen::Vector3i> z_offsets{ Eigen::Vector3i(0,0,1), Eigen::Vector3i(0,0,0), Eigen::Vector3i(0,0,-1) } ;
		double mannhattanDistance(Eigen::Vector3i& p1, Eigen::Vector3i& p2) {
			return fabs(p1(0)-p2(0)) + fabs(p1(1)-p2(1)) + fabs(p1(2) - p2(2)) ;
		}
		
		double euclideanDistance(Eigen::Vector3i& p1, Eigen::Vector3i& p2) {
			double x, y, z ;
			x = p1(0) - p2(0) ;
			y = p1(1) - p2(1) ;
			z = p1(2) - p2(2) ;
			return sqrt( x*x + y*y + z*z ) ;
		}	

		double costToGo(Eigen::Vector3i& p1, Eigen::Vector3i& p2) {
			if (ogrid_->isOccupied(p2) ) return HIGH_COST ;
			return 1 ; // This may change if cost to go for non occupied cell is identified to matter.
		}

		double radial_tolerance ;
		int getKey(Eigen::Vector3i& point) { return ogrid_->posToIndex(point) ; }
		int getKey(WayPoint& wp) { return getKey( wp.getPoint() ) ; }
		bool isOccupied(Eigen::Vector3i& point) { return ogrid_->isOccupied(point) ; }
};

}

#endif
