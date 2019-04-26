#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stdlib.h>
#include <vector>

namespace waypoint
{


struct Key{
	double k[2] ;

	bool operator() (const Key& k1, const Key& k2) {
		if (k1.k[0] < k2.k[0]) return true ;
		return (k1.k[0] == k2.k[0] && k1.k[1] <= k2.k[1]) ;
	}

	bool operator ==(const Key& k1) {
		return (k1.k[0] == k[0] && k1.k[1] == k[1]) ;
	}
};

const int NEW = 0 ;
const int OPEN = 1 ;
const int CLOSED = 2 ;
const int RAISED = 3 ;
const int LOWERED = 4 ;
const int REMOVED = -1 ;

bool operator ==(const Key& k1, const Key& k2) {
	return (k1.k[0] == k2.k[0] && k1.k[1] == k2.k[1]) ;
}

bool operator <=(const Key& k1, const Key& k2) {
	if (k1.k[0] < k2.k[0]) return true ;
	return (k1.k[0] == k2.k[0] && k1.k[1] <= k2.k[1]) ;
}

bool operator !=(const Key& k1, const Key& k2) {
	return (k1.k[0] != k2.k[0] && k1.k[1] != k2.k[1]) ;
}

bool operator <(const Key& k1, const Key& k2) {
	return (k1.k[0] < k2.k[0] && k1.k[1] < k2.k[1]) ;
}

class WayPoint
{

	public:
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward, double min_cost, double curr_cost) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward,  double min_cost ) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward,  Key& k ) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, WayPoint* backward ) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, double min_cost ) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward, Key& k ) ;
		WayPoint(Eigen::Vector3i& point, WayPoint* forward ) ;
		WayPoint(Eigen::Vector3i& point, Key& k ) ;
		WayPoint(Eigen::Vector3i& point ) ;
		WayPoint() ;

		double getMinCost() { return min_cost_; }
		double getCurrCost() { return curr_cost_; }
		std::vector<Eigen::Vector3i> getNeighbors() { return neighbors_; }
		WayPoint* getForwardPt() { return forward_; }
		WayPoint* getBackPt() { return back_; }
		Eigen::Vector3i& getPoint() { return point_; }
		Key getKey() const { return k_ ; }
		
		void setForwardPt(WayPoint* forward) { forward_ = forward; }
		void setBackPt(WayPoint* backward) { back_ = backward; }
		void setPoint(Eigen::Vector3i& point) { point_ = point; } 

		WayPoint* getForward() { return forward_ ; }
		WayPoint* getBackward() { return back_ ; }

		bool operator !=(WayPoint& other) { return (point_ != other.getPoint() && k_ != other.getKey() ); }
		bool operator ==(WayPoint& other) { return (point_ == other.getPoint() && k_ == other.getKey() ); }
		bool operator ==(Eigen::Vector3i& other) {return point_ == other; }
		bool operator <=(const Key kp) {
			if( k_.k[0] < kp.k[0] ) return true;
			return ( k_.k[0] == kp.k[0] && k_.k[1] <= kp.k[1]) ;
		}

		bool operator () (WayPoint& wp1, WayPoint& wp2) {
			return wp1.getKey() <= wp2.getKey() ;
		}

		WayPoint operator =(WayPoint other) {
			point_ = other.getPoint() ;
			forward_ = other.getForwardPt() ;
			back_ = other.getBackPt() ;
			state = other.getState() ;
			k_ = other.getKey() ;

			return *this ;
		}

//		WayPoint& operator =(WayPoint& other) {
//			point_ = other.getPoint() ;
//			forward_ = other.getForwardPt() ;
//			back_ = other.getBackPt() ;
//			state = other.getState() ;
//			k_ = other.getKey() ;
//			
//			return *this ;
//		}
		
		void setMinCost(double new_cost) { min_cost_ = new_cost; }
		void setCurrCost(double new_cost) { curr_cost_ = new_cost; }
		
		double getG() { return k_.k[0] ; }
		double getRHS() { return k_.k[1]; }
		int getState() { return state ; }	

		void setG(double val) { k_.k[0] = val; }
		void setRHS( double val) { k_.k[1] = val; }
		void setKey( Key& k ) { k_ = k; } 
		void setState( int new_state ) { state = new_state; }
	
	private:
		Eigen::Vector3i& point_ ;
		WayPoint* forward_, *back_ ;
		double min_cost_ ;
		double curr_cost_ ;
		//double k_[2] ;
		int state; // Possible states: 0 - New, 1 - Open, 2-Closed, 3-Raised, 4-Lowered
		Key k_ ;
		std::vector<Eigen::Vector3i> neighbors_ ;
		void makeNeighbors() ;
};

bool operator <(WayPoint& wp1, WayPoint& wp2) {
	return wp1.getKey() < wp2.getKey() ;
}

}

#endif
