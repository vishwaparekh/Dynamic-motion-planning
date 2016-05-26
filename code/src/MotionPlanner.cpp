#include "MotionPlanner.hpp"
#include "Graphics.hpp"
#include <math.h>
#include <list>
#include <algorithm>
#include <map>
#include <iostream>

using namespace std;

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;
	
    prm_nr_vertices = NR_VERTICES_PRM;
    prm_nr_neighbours = 18;

    prm_sampled = false;
    prm_connected = false;

    prm_nr_neighbours_query = prm_nr_vertices;
    prm_found_near_robot = false;

    isPotential = false;
    inTransit = true;
    
    step_path = false;
    step_size_path = 0.1;
    
    current_path_index = -1;
    next_path_index = -1;
}

MotionPlanner::~MotionPlanner(void)
{
}

bool MotionPlanner::Solve()
{   
    //stores the value of robot configuration
    double robot_x = m_simulator->GetRobotCenterX();
    double robot_y = m_simulator->GetRobotCenterY();

    /////////////////////////
    // Roadmap construction
    
    //get the boundaries of bounding box
    double bound_max_x = m_simulator->GetBoundingBoxMax()[0];
    double bound_max_y = m_simulator->GetBoundingBoxMax()[1];
    double bound_min_x = m_simulator->GetBoundingBoxMin()[0];
    double bound_min_y = m_simulator->GetBoundingBoxMin()[1];
    
    int nStaticObstacles = m_simulator->GetNrStaticObstacles();
    
    int k = 0;
    // loop until given number of vertices have been sampled
    while(k < prm_nr_vertices)
    {
	    bool inCollision = false;
		//will store the new robot configuration
		double new_config_x = PseudoRandomUniformReal(bound_min_x,bound_max_x);
		double new_config_y = PseudoRandomUniformReal(bound_min_y,bound_max_y);

		// check for collision with all static obstacles
		for(int iter_obs = 0; iter_obs < nStaticObstacles; iter_obs++)
		{
		    const int nv   = m_simulator->GetObstacleNrVertices(iter_obs);
		    const double *poly = m_simulator->GetObstacleVertices(iter_obs);

            // check if the vertex is inside the obstacle
            if (IsPointInsidePolygon(new_config_x, new_config_y, nv, poly)){
                inCollision = true;
                break;
            }

            // check if the robot at this vertex is in intersection with the obstacles
            if (CirclePolygonIntersection(new_config_x, new_config_y, m_simulator->GetRobotRadius(), nv, poly)){
                inCollision = true;
                break;
            }
		}

        // add to the vertex list the new position
        if(!inCollision){
            prm_vertices.push_back(new_config_x);
            prm_vertices.push_back(new_config_y);
            k++;
		}		
    }

    // add the goal center also to the PRM vertex list
    prm_vertices.push_back(m_simulator->GetGoalCenterX());
    prm_vertices.push_back(m_simulator->GetGoalCenterY());
    prm_nr_vertices++;
    prm_sampled = true;
    
    // connect the PRM vertices to form the PRM edges
    prm_edges.resize(prm_nr_vertices);
    for (int i = 0; i < prm_nr_vertices; i++){
        GetNeighboursByDirectInterpolation(i);
    }
    prm_connected = true;
    printf("Connected.\n");

    // PRM has been constructed and set-up

    // Run Djiksta's algorithm for computing the weights for all the vertices.
    GetWeightsByDijkstra(prm_nr_vertices - 1);

    // Transition the robot from its current position to the closest PRM vertex found
    current_path_index = GetOptimumPRMVertex(robot_x, robot_y);
    // if no optimal vertex found function call returned with error value.
    if (current_path_index == -1){
        return false;
    }
    current_path_slope = atan2(prm_vertices[2*current_path_index + 1] - m_simulator->GetRobotCenterY(),
        prm_vertices[2*current_path_index] - m_simulator->GetRobotCenterX());
    prm_found_near_robot = true;
    printf("Transiting towards start point on PRM : %d\n", current_path_index);

    return true;
}

void MotionPlanner::moveRobot(void)
{
    // if robot is travelling on the PRM
	if(!isPotential && !inTransit)
	{
		if (!step_path){
            // if next_path_index needs to be retrieved
            next_path_index = prm_vertex_parents[current_path_index];
            current_path_slope = atan2((prm_vertices[2*next_path_index + 1] - prm_vertices[2*current_path_index + 1]),
                (prm_vertices[2*next_path_index] - prm_vertices[2*current_path_index]));
            step_path = true;
        }

        // get new point(x1,y1) alont the PRM edge
        double x1 = m_simulator->GetRobotCenterX() + cos(current_path_slope) * step_size_path;
        double y1 = m_simulator->GetRobotCenterY() + sin(current_path_slope) * step_size_path;
        // check if reached the next vertex
        if (EuclideanDistance(x1, y1, prm_vertices[2*next_path_index], prm_vertices[2*next_path_index + 1]) < 0.1){
            current_path_index = next_path_index;
            step_path = false;
            // set the robot's position to be next vertex
            m_simulator->SetRobotPosition(prm_vertices[2*current_path_index], prm_vertices[2*current_path_index + 1]);
            printf("Reached PRM vertex: %d.\n", current_path_index);
        }
        else{
            // set the robot's position to the step values computed
            m_simulator->SetRobotPosition(x1, y1);
        }

        // check if any hits found by the laser nearby
        if(m_simulator->ScanSurrounding()) isPotential = true;
	}
    // if obstacle encounterd and potential functions need to be used
	else if(isPotential && !inTransit)
	{
		double x_hit, y_hit;
		bool moving = false;
		double robot_x = m_simulator->GetRobotCenterX();
		double robot_y = m_simulator->GetRobotCenterY();
        double robot_rad = m_simulator->GetRobotRadius();
		
        // step defined using the potential functions
        double grad_u_rep_final_x = 0;
		double grad_u_rep_final_y = 0;

        // counter to count consecutive occurrence of NO HIT angles
        int count_free_angle = 0;
        // flag to indicate if the repusive force has overshot a threshold value
        bool rep_overshoot = false;
        // storse the angle mdiway between the free angles where there is no 
        // detection of hits
        double free_angle = -1;

        // iterate through all the laser angles to detect hits
        for(int n = 0; n < NR_LASER_SCAN_ANGLES; n++){
            double phi = 2*n*M_PI/NR_LASER_SCAN_ANGLES;
            moving |= m_simulator->TakeStaticLaserReading(phi, x_hit, y_hit);
            
            // if there has been a hit either with a moving or a static obstacle
            if (x_hit != MAXINT && y_hit != MAXINT){
                count_free_angle = 0;
			    double dx = 0, dy = 0;
                // distance of the hit point from the robot position
			    double dist_ob = EuclideanDistance(robot_x, robot_y, x_hit, y_hit) - robot_rad;
			    double theta = atan2((y_hit - robot_y),(x_hit - robot_x));
			
                //check if dist_ob is less than threshold distance for repulsion
                double threshold_dist = ((double)NR_LASER_SCAN_STEPS)/10;
                
                // compute the repulsive gradient force
                if(dist_ob < threshold_dist){
                    grad_u_rep_final_x  += (((1/threshold_dist)-(1/dist_ob))*(1/(dist_ob*dist_ob)))*cos(theta);
                    grad_u_rep_final_y  += (((1/threshold_dist)-(1/dist_ob))*(1/(dist_ob*dist_ob)))*sin(theta);
                }
                // check if the distance between the robot and hit point is within a minimum distance
                if (dist_ob <= 0.25*threshold_dist){
                    rep_overshoot = true;
                }
            }
            else{
                count_free_angle++;              
            }
            // if we have more than 5 consecutive free angular regions we store the mdidle angle
            // so that we can use this to bail out in case of a local minima occuring
            if (count_free_angle >= 5){
                    free_angle = (n - count_free_angle/2) * 2 * M_PI / NR_LASER_SCAN_ANGLES;
            }
		}
        // if the repulsive gradient might have overshot beacause of priximity between the hit
        // and the robot position, we add another force computed with the free_angle found
        if (rep_overshoot){
            grad_u_rep_final_x += EuclideanDistance(0, 0, grad_u_rep_final_x, grad_u_rep_final_y) * cos(free_angle);
            grad_u_rep_final_y += EuclideanDistance(0, 0, grad_u_rep_final_x, grad_u_rep_final_y) * sin(free_angle);
        }
		
        // cap the gradient values on the top and bottom
        double max_grad_step = robot_rad ;
        if(grad_u_rep_final_x > max_grad_step) grad_u_rep_final_x = max_grad_step;
		if(grad_u_rep_final_y > max_grad_step) grad_u_rep_final_y = max_grad_step;
		if(grad_u_rep_final_x < -max_grad_step) grad_u_rep_final_x = -max_grad_step;
		if(grad_u_rep_final_y < -max_grad_step) grad_u_rep_final_y = -max_grad_step;
		
        robot_x += grad_u_rep_final_x;
		robot_y += grad_u_rep_final_y;
        // set the robot position to the nex position set
		m_simulator->SetRobotPosition(robot_x,robot_y);
		
        // if no hit with moving obstacle then compute the optimal point next to 
        if(!moving)
		{
			isPotential = false;
            inTransit = true;
            current_path_index = GetOptimumPRMVertex(robot_x, robot_y);
            current_path_slope = atan2(prm_vertices[2*current_path_index + 1] - m_simulator->GetRobotCenterY(),
                prm_vertices[2*current_path_index] - m_simulator->GetRobotCenterX());
            printf("Transiting towards PRM vertex : %d\n", current_path_index);
        }
	}
    // if the robot is outside the PRM and needs to get inside onto the PRM
    else if (inTransit){
        // if surrounding has obstace then 
        if(m_simulator->ScanSurrounding()) {
            isPotential = true;
            inTransit = false;
        }
        double x1 = m_simulator->GetRobotCenterX() + cos(current_path_slope) * step_size_path;
        double y1 = m_simulator->GetRobotCenterY() + sin(current_path_slope) * step_size_path;
        if (EuclideanDistance(x1, y1, prm_vertices[2*current_path_index], prm_vertices[2*current_path_index + 1]) < 0.1){
            inTransit = false;
            step_path = false;
            m_simulator->SetRobotPosition(prm_vertices[2*current_path_index], prm_vertices[2*current_path_index + 1]);
            printf("Reached PRM vertex: %d.\n", current_path_index);
        }
        else{
            m_simulator->SetRobotPosition(x1, y1);
        }
    }
}

void MotionPlanner::Draw(void)
{   
    // draw the vertices when the PRM vertices have been sample.
    if (prm_sampled){
        for (int i = 0; i < prm_nr_vertices; i++){
            DrawColor(1, 0, 0);
            DrawPoint2D(prm_vertices[2*i], prm_vertices[2*i+1]);
        }
    }
    // draw the edges once the PRM has been connected.
    if (prm_connected){
        for (int i = 0; i < prm_nr_vertices; i++){
            DrawColor(0.8, 0.8, 0.8);
            double x1 = prm_vertices[2*i];
            double y1 = prm_vertices[2*i + 1];
            for (list<int>::iterator iter = prm_edges[i].begin(); iter != prm_edges[i].end(); iter++){
                DrawSegment2D(x1, y1, prm_vertices[2*(*(iter))], prm_vertices[2*(*iter) + 1]);
            }
        }
    }
    // set once the robot has found the nearest PRM point to it
    if (prm_found_near_robot){
        if (current_path_index != -1){
            DrawColor(0.1, 0.33, 0.1);
            DrawPoint2D(prm_vertices[2*current_path_index], prm_vertices[2*current_path_index+1]);
        }
        if (next_path_index != -1){
            DrawColor(0.86, 0.12, 0.86);
            DrawPoint2D(prm_vertices[2*next_path_index], prm_vertices[2*next_path_index+1]);
        }
        if (current_path_index != -1 && next_path_index != -1){
            DrawColor(0.88, 0.4, 0.11);
            DrawSegment2D(prm_vertices[2*current_path_index], prm_vertices[2*current_path_index+1],
                prm_vertices[2*next_path_index], prm_vertices[2*next_path_index+1]);
        }
    }
    
}

void MotionPlanner::GetNeighboursByDirectInterpolation(const int index){

    double x = prm_vertices[2*index];
    double y = prm_vertices[2*index + 1];

    // map used so that the key [distance] will be getting sorted as and when we end
    map<double, int> dist;
    // compute euclidean distance between my current vertex and every other vertex and store the result in sorted order in dist
    for (int i = 0; i < prm_vertices.size()/2; i++){
        dist[EuclideanDistance(x, y, prm_vertices[2*i], prm_vertices[2*i + 1])] = i;
    }
    //now for the first 'k = prm_nr_neighbors', check if a collision free path is possible. If possible add them to the neighbours for the current vertex
    map<double, int>::iterator iter = dist.begin();
    for (int k = 0; k < prm_nr_neighbours; k++, iter++){   
        if (iter->second == index) continue;
        if (IsLocalPathSimple(x, y, prm_vertices[2*iter->second], prm_vertices[2*iter->second + 1])){
            prm_edges[index].push_back(iter->second);
        }

    }

}
//function to check if a collision free path is possible between start and end
bool MotionPlanner::IsLocalPathSimple(const double start_x, const double start_y, const double end_x, const double end_y){
    //calculate slope between the start point and the end point
    double theta = atan2(end_y - start_y, end_x - start_x);
    double x = start_x; double y = start_y;
    //size of steps in which we want to check for collision along the edge i.e we basically divide the edge into number of small edges of 0.1 length each. Check
    //for collisions at end of each of these small edges.
    double step_size_local_path = 0.1;
    double length = EuclideanDistance(end_x, end_y, start_x, start_y);
    //check for collision with all the static obstacles and just the segment
    for (int i = 0; i < m_simulator->GetNrStaticObstacles(); i++){
        int nv = m_simulator->GetObstacleNrVertices(i);
	    const double *poly = m_simulator->GetObstacleVertices(i);
        if (SegmentPolygonIntersection(start_x, start_y, end_x, end_y, nv, poly)){
            return false;
        }
    }
    //iterate through the length as explained above and check for robot polygon collisions for all the obstacles
    while(EuclideanDistance(x, y, start_x, start_y) < length){
        x = x + step_size_local_path * length * cos(theta);
	    y = y + step_size_local_path * length * sin(theta);
        for (int i = 0; i < m_simulator->GetNrStaticObstacles(); i++){
            int nv = m_simulator->GetObstacleNrVertices(i);
	        const double *poly = m_simulator->GetObstacleVertices(i);
	    	
            if (CirclePolygonIntersection(x, y, m_simulator->GetRobotRadius(), nv, poly)){
                return false;
            }
        }
    }
    return true;
}
//function to get optimum PRMVertex when in transit mode
int MotionPlanner::GetOptimumPRMVertex(const double x, const double y)
{
    //map data structure used to keep the distances in sorted order
    map<double, int> net_dist;
    //calculate the euclidean distance + weight of vertex to which euclidean distance is being calculated and store them in net_dist
    for (int i = 0; i < prm_nr_vertices; i++){
        net_dist[prm_vertex_weights[i] + EuclideanDistance(x, y, prm_vertices[2*i], prm_vertices[2*i + 1])] = i;
    }
    //find the best match from net_dist which has a collision free best path from current point to goal, using Dijkstra weights of the vertices used in going towards the goal
    map<double, int>::iterator iter = net_dist.begin();
    int k = 0;
    bool nearest_neighbour_found = false;
    while(k < prm_nr_neighbours_query){    
        if (IsLocalPathSimple(x, y, prm_vertices[2*iter->second], prm_vertices[2*iter->second + 1])){
            nearest_neighbour_found = true;
            break;
        }
        k++;
        iter++;
    }

    if (nearest_neighbour_found) return iter->second;
    else return -1;
}

int MotionPlanner::GetNearestNeighbourByDirectInterpolation(const double x, const double y){
    //stores the Euclidean distance values in a sorted order
    map<double, int> dist;

    for (int i = 0; i < prm_nr_vertices; i++){
        dist[EuclideanDistance(x, y, prm_vertices[2*i], prm_vertices[2*i + 1])] = i;
    }
    //from the list, find the nearest vertex to which collision free path is possible
    map<double, int>::iterator iter = dist.begin();
    int k = 0;
    bool nearest_neighbour_found = false;
    while(k < prm_nr_neighbours_query){    
        if (IsLocalPathSimple(x, y, prm_vertices[2*iter->second], prm_vertices[2*iter->second + 1])){
            nearest_neighbour_found = true;
            break;
        }
        k++;
        iter++;
    }

    if (nearest_neighbour_found) return iter->second;
    else return -1;
}

void MotionPlanner::GetWeightsByDijkstra(const int start){
    //keep a vector of visited vertices. initially assign all the vertices to the value - false i.e. not visited
    std::vector<bool> visited_vertices;
    visited_vertices.assign(prm_nr_vertices, false);
    //list of vertices that have not been visited. push all the vertices into this vector except the goal
    std::list<int> unvisited_vertices;
    for (int i = 0; i < prm_nr_vertices; i++){
        if (i != start) unvisited_vertices.push_back(i);
    }
    //to save all the lengths in a sorted order
    std::vector<map<int,double> > lengths;
    //go through the lenghts of all the neighbors for every vertex and save the values of the euclidean distances
    lengths.resize(prm_nr_vertices);
    for (int i = 0; i < prm_nr_vertices; i++){
        std::list<int>::iterator iter = prm_edges[i].begin();
        while(iter != prm_edges[i].end()){
            lengths[i][*iter] = EuclideanDistance(prm_vertices[2*i], prm_vertices[2*i + 1], prm_vertices[2*(*iter)], prm_vertices[2*(*iter) + 1]);
            iter++;
        }
    }
    //initialize the parent array, this array will store the index of the parent node to the current node
    prm_vertex_parents.resize(prm_nr_vertices);
    for (int i = 0; i < prm_nr_vertices; i++){
        prm_vertex_parents[i] = MAXINT;
    }
    //initialize current vertex to start and initialize the vector of weights
    int curr_vertex = start;
    prm_vertex_weights.resize(prm_nr_vertices);
    prm_vertex_weights.assign(prm_nr_vertices, MAXINT);
    prm_vertex_weights[start] = 0;
    //repeat until not empty. Here what we do is first remove the vertex with lowest weight from unvisited list. For this node, remove all its unvisited neighbors and assign
    // them weights as w1 = w + length (v,v1) where v is current vertex, v1 = neighbors, w = current vertex weight and w1 neighbor weights. After this assign the current
    // vertex as parent to all its unvisited neighbors
    while(!unvisited_vertices.empty()){
        //cerr << curr_vertex << endl;
        std::list<int>::iterator iter1 = prm_edges[curr_vertex].begin();
        while(iter1 != prm_edges[curr_vertex].end()){
            if (!visited_vertices[*iter1]){
                if (prm_vertex_weights[*(iter1)] > prm_vertex_weights[curr_vertex] + lengths[curr_vertex][*(iter1)]){
                    prm_vertex_weights[*(iter1)] = prm_vertex_weights[curr_vertex] + lengths[curr_vertex][*(iter1)];
                    prm_vertex_parents[*iter1] = curr_vertex;
                }
            }
            iter1++;
        }
        
        visited_vertices[curr_vertex] = true;

        double min_weight = MAXINT;
        int min_weight_index = -1;
        //remove the next minimum vertex with lowest weight from the list
        for (std::list<int>::iterator iter2 = unvisited_vertices.begin(); iter2 != unvisited_vertices.end(); iter2++){
            if (*iter2 == curr_vertex){
                iter2 = unvisited_vertices.erase(iter2);
            }
            if (iter2 == unvisited_vertices.end()) break;
            if (prm_vertex_weights[*iter2] < min_weight){
                min_weight = prm_vertex_weights[*iter2];
                min_weight_index = *iter2;
            }
        }

        curr_vertex = min_weight_index;
        if (curr_vertex == -1) break;
    }
}
// Reset PRM
void MotionPlanner::ResetPRM(){
    
    prm_nr_vertices = NR_VERTICES_PRM;

    prm_vertices.clear();
    prm_edges.clear();

    prm_sampled = false;
    prm_connected = false;

    prm_vertex_weights.clear();
    prm_vertex_parents.clear();

    prm_found_near_robot = false;

    isPotential = false;
    inTransit = true;
    step_path = false;
    current_path_index = -1;
    next_path_index = -1;

    m_simulator->SetupGoal();
    m_simulator->SetupRobot();
}
