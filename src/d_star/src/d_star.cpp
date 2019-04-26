//#include "../include/d_star/d_star.h"
#include "d_star/d_star.h"
#include "d_star/path.h"

namespace d_star
{


	// TODO:: Write implementation for initilization and possibly overload initialization	
	DStar::DStar(OccupancyGrid* ogrid, Eigen::Vector3i& starting_point, Eigen::Vector3i& ending_point) {}

	
	bool DStar::isRaise(WayPoint point) {}
	double DStar::costVia(WayPoint current_point, WayPoint next_point) {} 		
	double DStar::G(Eigen::Vector3i& point) 
	{
		//TODO: Determine if this function is necessary
		return 0 ;
	}
	double DStar::heuristic(Eigen::Vector3i& p1, Eigen::Vector3i& p2)
	{
		return mannhattanDistance(p1, p2) ;
		// return euclideanDistance(p1, p2) ; // Determine which cost function works best...
	} 
	
	double heuristic(WayPoint wp1, WayPoint wp2) {
		return heuristic(wp1.getPoint(), wp2.getPoint()) ;
	}	
	
	double DStar::rhs(Eigen::Vector3i& p1, Eigen::Vector3i& p2) 
	{
		if (p1 == sp) return 0 ;
		double occupancy = isOccupied(p2) ; 
		//return heuristic(p2, gp) + occupancy * HIGH_COST + 1  ; // distance from start to p2 + cost if point2 is an obstacle + cost to go
		return heuristic(p2, sp) + costToGo(p1, p2) ;
	}

	//Key calculateKey(Eigen::Vector3i& point)
	Key DStar::calculateKey(Eigen::Vector3i& point)
	{
		int key = getKey(point) ; 

		double val = fmin( table[key].getG(), table[key].getRHS() ) ;
		Key tmp ;
		tmp.k[0] = val + heuristic(point, sp) + km ;
		tmp.k[1] = val ;

		return tmp ;
	}

	/*
		Implementation of the Initialize function as outlined in the D* Lite paper [Koeing, Likhachev]
	*/
	void DStar::initialize() 
	{
		//TODO: Create a list that will contain the waypoints in the open list
		while( !Q.empty() ){
			 Q.pop() ; // Empty the priority queue if it is not empty
		}
		km = 0;
		Key tmp ;
		tmp.k[1] = 0 ;
		tmp.k[0] = heuristic(sp, gp) ;
		WayPoint g_wp = WayPoint(gp, tmp) ;		

		tmp.k[1] = tmp.k[0] ;
		tmp.k[0] = 0 ;
		WayPoint s_wp = WayPoint(sp, tmp) ;
		table[getKey(gp)] = g_wp ;
		table[getKey(sp)] = s_wp ;
	
		Q.push(g_wp) ; // Insert the goal waypoint into the open list		
		
		// ROS related initilization:

		path_pub = p_nh.advertise<d_star::path>("D_Star_Path", 1) ;
	}

	void DStar::newCell(WayPoint& wp) {
		int key = getKey(wp) ;
		if (table.find(key) != table.end()) return ;

		Key tmp ;
		tmp.k[0] = tmp.k[1] = heuristic(wp.getPoint(), gp) ;
		wp.setKey(tmp) ;
		table[key] = wp ;
	}

	void DStar::update(WayPoint& wp) {
		update(wp, 1) ;
	}

	void DStar::update(WayPoint& wp, int ins) {
		Key tmp = calculateKey(wp) ;
		if (tmp != wp.getKey() ) wp.setKey(tmp) ;
		int key = getKey(wp) ;
		wp.setState( OPEN ) ;
		table[key] = wp ;
		if (ins == 1) {
			Q.push(wp) ;
		}
	}

	void DStar::insert(WayPoint& wp, int ins) {
		newCell(wp) ;
		update(wp, ins) ;
	}

	void DStar::remove(WayPoint& wp) {
		int key = getKey(wp) ;
		if (table.find(key) == table.end()) return ;
		//table.erase(key) ;
		table[key].setState(REMOVED) ;
	}

	void DStar::updateVertex(WayPoint& wp1, WayPoint& wp2) 
	{
		int key = getKey(wp1) ;
		WayPoint* temp = &table[key] ;
		wp2.setForwardPt( temp ) ;
		updateVertex(wp2) ;
	}
	void DStar::updateVertex(WayPoint& wp) 
	{
		Key tmp = wp.getKey() ;
		if (wp.getG() != wp.getRHS() && wp.getState() == OPEN) update(wp) ;		
		else if (wp.getG() != wp.getRHS() && wp.getState() != OPEN) insert(wp) ;
		else if (wp.getG() == wp.getRHS() && wp.getState() == OPEN) remove(wp) ;

	}

	WayPoint DStar::top() 
	{
		WayPoint curr_wp = Q.top() ;
		WayPoint query_wp ;
		int key = getKey(curr_wp) ; 
		query_wp = table[key] ;
		while (curr_wp != query_wp || query_wp.getState() == CLOSED || query_wp.getState() == REMOVED) {
			Q.pop() ;
			WayPoint curr_wp = Q.top() ;
			key = getKey(curr_wp) ;
			query_wp = table[key] ;
		}

		return curr_wp ;
	}

	WayPoint DStar::getPoint(Eigen::Vector3i& point, WayPoint& u) {
	
		int key = getKey(point) ;		
		if (table.find(key) == table.end()) {
			WayPoint* tmp = &u ;
			WayPoint new_wp = WayPoint(point, new WayPoint(), tmp) ;
			insert(new_wp) ;
		}
		return table[key] ;
	}

	WayPoint DStar::getPoint(Eigen::Vector3i& point) 
	{
		int key = getKey(point) ;
		return table[key] ;
	}

	double DStar::rhs(WayPoint wp) {
		if( wp.getPoint() == sp )return 0 ;
	
		double min_cost = HIGH_COST ;
		for (std::vector<Eigen::Vector3i>::iterator it = z_offsets.begin(); it != z_offsets.end(); ++it) {
			for (std::vector<Eigen::Vector3i>::iterator direc = new_directions.begin(); direc != new_directions.end(); ++direc) {
				Eigen::Vector3i new_p = wp.getPoint() + *direc + *it ;
				if( new_p == wp.getPoint()) continue;
				WayPoint curr_wp = getPoint(new_p, wp) ;
				double curr_cost = costToGo(wp.getPoint(), new_p); 
				if (curr_cost < min_cost) min_cost = curr_cost ;
			}
		}
		return min_cost ;
	}

	void DStar::computeShortestPath() 
	{
		start = table[ getKey(sp) ] ;
		while(top().getKey() < calculateKey(sp) || start.getRHS() > start.getG() ) {
			WayPoint u = top() ;
			int key = getKey( u ) ;
			Key kold = u.getKey() ;
			Key knew = calculateKey(u) ;
			
			if (kold < knew) {
				update(u) ;	
			} else if (u.getG() > u.getRHS() ) {
				u.setG(u.getRHS()) ;
				remove(u) ;
				for (std::vector<Eigen::Vector3i>::iterator it = z_offsets.begin(); it != z_offsets.end(); ++it) {
					for (std::vector<Eigen::Vector3i>::iterator direc = new_directions.begin(); direc != new_directions.end(); ++direc) {
						Eigen::Vector3i new_p = u.getPoint() + *direc + *it ;
						if( new_p == u.getPoint()) continue ;
						WayPoint new_wp = getPoint(new_p, u) ;
						if (sp != new_p) new_wp.setRHS(std::fmin(new_wp.getRHS(), costToGo(u.getPoint(), new_p) +new_wp.getG() ) ) ;
						updateVertex(new_wp, u) ;
					}
				}
			} else {
				double gold = u.getG();
				u.setG(HIGH_COST) ;
				for (std::vector<Eigen::Vector3i>::iterator it = z_offsets.begin(); it != z_offsets.end(); ++it) {
					for (std::vector<Eigen::Vector3i>::iterator direc = new_directions.begin(); direc != new_directions.end(); ++direc) {
						Eigen::Vector3i new_p = u.getPoint() + *direc + *it ;
						WayPoint new_wp = getPoint(new_p, u) ;
						if (new_wp.getRHS() == costToGo(u.getPoint(), new_p) + gold or new_wp == u ) {
							if (new_p != sp) new_wp.setRHS(rhs(new_wp)) ;
						}
						updateVertex(new_wp, u) ;
					}			
				}
			}
			//table[key].setState(REMOVED) ;
			//Q.pop()
		}
		sendPath() ;
	}

	bool DStar::obstacleDetected(WayPoint& wp) {
		
		WayPoint* next_wp = &wp ;
		for( int i = 0; i < look_ahead_distance; i++) {
			WayPoint* next_wp = next_wp->getForward() ;
			int key = getKey( next_wp->getPoint() ) ; 
			if (isOccupied( table[key].getPoint() )) return true ;
		}
	
		return false ;
	}

	void DStar::Main() {
		initialize() ;
		computeShortestPath() ;
		sgoal = table[ getKey(gp) ] ;
		start = table[ getKey(sp) ] ;
		slast = start;
		while(start != sgoal ) {
			//start = getPoint(start.getForwardPt()->getPoint()) ;
			Eigen::Vector3i& tmp_point = start.getForwardPt()->getPoint() ;
			start = getPoint( tmp_point ) ;
			// WAIT TILL THE CURRENT LOCATION IS START OR A FEW POINTS AHEAD OF START
			//double dist = euclideanDistance(start.getPoint(), ogrid_->octPosToGrid(curr_pos - oct_root_pos) ) ;
			Eigen::Vector3i curr_grid_pos = ogrid_->octPosToGrid(curr_pos - oct_root_pos) ;
			while( euclideanDistance(start.getPoint(), curr_grid_pos) > radial_tolerance) {
				curr_grid_pos = ogrid_->octPosToGrid(curr_pos - oct_root_pos) ;
				// CREATE A PYTHON NODE THAT PERIODICALLY PUBLISHES THE END EFFECTOR STATE
				
			}

			//if obstacle_detected {
			if( obstacleDetected(start)) {
				km += heuristic(slast, start) ;
				slast = start ;

				WayPoint* next_wp = &start ;
				for( int i = 0; i < look_ahead_distance; i++) {
					WayPoint* curr_wp = next_wp ;
					next_wp = next_wp->getForward() ;
					double ctg = costToGo(curr_wp->getPoint(), next_wp->getPoint()) ;
					if ( 1 > ctg ) {
						if (start != sgoal) next_wp->setRHS( fmin( next_wp->getRHS(), ctg + next_wp->getG() ) ) ;
					} else if (next_wp->getRHS() == 1 + next_wp->getG() ) {
						if (start != sgoal) next_wp->setRHS( rhs( *next_wp ) ) ;
					}
					updateVertex(*next_wp, *curr_wp) ;
				}
				computeShortestPath() ;
			}
		}
	}

	Eigen::Vector3d DStar::gridToOct( Eigen::Vector3i point ) 
	{
		return ogrid_->gridPosToOcto( point ) ;
	}

	Eigen::Vector3d DStar::gridToOct( WayPoint wp ) 
	{
		return gridToOct( wp.getPoint() ) ;
	}

	void DStar::sendPath() 
	{
		//geometry_msgs::Point[] path ;
		d_star::path msg ;
		WayPoint* next_wp = &start ;
		while( next_wp->getForwardPt() != nullptr ) {
			int key = hashKey( *next_wp ) ;
			Eigen::Vector3d point = gridToOct( table[key] ) ;
			//path.append( Point( point.x, point.y, point.z) ) ;
			geometry_msgs::Point tmp_point ;
			tmp_point.x = point(0) ;
			tmp_point.y = point(1) ;
			tmp_point.z = point(2) ;
			msg.path.push_back( tmp_point ) ;
			next_wp = next_wp->getForwardPt() ;
		}

		msg.header.stamp = ros::Time::now() ;
		path_pub.publish( msg ) ;
	}
}
