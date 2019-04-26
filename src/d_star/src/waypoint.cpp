#include "d_star/waypoint.h"

namespace waypoint 
{

const double max_d = 100000 ;
const std::vector<Eigen::Vector3i> offsets{Eigen::Vector3i(1,1,1), Eigen::Vector3i(1,-1,1), Eigen::Vector3i(-1,-1,1), Eigen::Vector3i(-1,1,1), Eigen::Vector3i(1,1,-1), Eigen::Vector3i(1,-1,-1), Eigen::Vector3i(-1,-1,-1), Eigen::Vector3i(-1,1,-1)} ;		

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward, double min_cost, double curr_cost) : 
  point_(point),
  forward_(forward),
  back_(backward), 
  min_cost_(min_cost), 
  curr_cost_(curr_cost)
{
  //makeNeigbors() ;
  k_.k[0] = min_cost ;
  k_.k[1] = curr_cost ;
  state = 0 ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward,  double min_cost ) 
{ 
  WayPoint(point, forward, NULL, min_cost, max_d) ; 
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward ) 
{
  WayPoint(point, forward, NULL, max_d, max_d) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, double min_cost ) 
{
  WayPoint(point, forward, NULL, min_cost, max_d) ;
}


WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward,  Key& k ) 
{
	WayPoint(point, forward, backward, k.k[0], k.k[1]) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward) 
{
	WayPoint(point, forward, backward, max_d, max_d) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward, Key& k ) 
{
	WayPoint(point, forward, NULL, k.k[0], k.k[1]) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, WayPoint* forward ) 
{
  WayPoint(point, forward, NULL, max_d, max_d) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point, Key& k) 
{
	WayPoint(point, NULL, NULL, k[0], k[1]) ;
}

WayPoint::WayPoint(Eigen::Vector3i& point ) 
{
  WayPoint(point, NULL, NULL, max_d, max_d) ;
}

void WayPoint::makeNeighbors() {
  // THERE SHOULD BE 26 NEIGHBORS AND NEED TO FIGURE OUT HOW TO NOT MAKE MULTIPLE OF THE SAME POINTS...
  for(std::vector<Eigen::Vector3i>::iterator it = offsets.begin(); it != offsets.end(); it++) {
    neighbors_.push_back(point + *it) ;
  }
}

}
