#include <list>
/**
 *@file MotionPlanner.hpp
 *@brief Interface for the motion-planning algorithms
 */

#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#define DEBUG 0							//Debug flag
#define CLOCKWISE 0						//State for turn mode to be CW
#define COUNTER_CLOCKWISE 1				//State for turn mode to be CCW
#define PIHALF 1.5707964				//90 degrees in radian
#define NR_VERTICES_PRM 100             // number of vertices to be sampled in PRM

#include "Simulator.hpp"

/**
 *@brief Interface for the motion-planning algorithms
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class MotionPlanner
{

public:
    
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    MotionPlanner(Simulator * const simulator);
                
    
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~MotionPlanner(void);
    
    /**
     * The graphical interface will call this function when the user selects the 
     * run motion planner option. This method creates the Probabilistic Road Map (PRM)
     * for the given environment using the static obstacles, goal and the robot.
     * This consists of sampling vertices, connecting them, running Dijkstra's algorithm
     * and finding the closest sampled vertex to the current robot position
     * On successfully setting the PRM, true value if returned. This true value then 
     * triggers the call to moveRobot from the graohical interface.
     */
    bool Solve();

    /**
     * Depending on the scenario in which the robot is currently this method computes
     * the new (next) position for the robot. The robot an either be freely moving on
     * the PRM or moving towards the PRM vertex or moving away from an oncoming obstacle.
     */
    void moveRobot(void);

    /**
     *@brief Draw motion planner
     *@remark
     *  You can use this function to draw your motion planner. It may help you during
     *  debugging to figure out how the motion planner is performing.
     *  You can use the functions from Graphics to do the drawing of points,
     *  edges, and so on.
     */
    void Draw();

protected:

    /**
     *@brief Pointer to simulator, which provides access to robot, obstacles, reward regions
     */
    Simulator  *m_simulator;

    /**
     * number of sample points to be considered while constructing the PRM
    **/
    int prm_nr_vertices;

    /**
     * list of the sample configurations for the PRM arranged in the following manner : x1, y1, x2, y2,... so on
     * size = 2 * prm_nr_vertices
    **/
    std::vector<double> prm_vertices;

    /**
     * number of close sample configurations which should be considered for computing the local paths
    **/
    int prm_nr_neighbours;

    /**
     * list of neighbour indices for every sample configuration present in the PRM.
     * every index corresponds to some configuration in the prm_vertices vector.
    **/
    std::vector<std::list<int> > prm_edges;

    /**
     * flag to indicate that the sampling configurations for the PRM have been found.
    **/
    bool prm_sampled;

    /**
     * flag to indicate that the sampled configurations have been connected to create the PRM.
    **/
    bool prm_connected;

    /**
     * number of close neighbours to consider while querying for closest point on PRM wrt a point not on PRM.
    **/
    int prm_nr_neighbours_query;
	
    /**
     * flag to indicate that a sample point has been found near the robot on the PRM.
    **/
    bool prm_found_near_robot;

    /**
     * stores teh weights computed using Dijkstra's algorithm
     */
    std::vector<double> prm_vertex_weights;

    /**
     * stores the parent vertex of each vertex in the PRM as assigned by Dijkstra's.
     */
    std::vector<int> prm_vertex_parents;

    /**
     * flag to indicate if potential fucntions need to be used to move the robot 
     * this happens when moving obstacles are encountered derailing the robot
     * from the PRM
     */
    bool isPotential;

    /**
     * flag to indicate that there is no obstacle in the range of the robot's laser
     * and that the robot is in the process of moving towards an optimal PRM vertex
     */
    bool inTransit;

    /**
     * stores the index of the vertex on the PRM where the robot may be transiting to
     * from outside the PRM or from where it is moving to the next vertex on the PRM
     */
    int current_path_index;

    /**
     * stores the index of the next vertex on the PRM that needs to be reached on the 
     * way to the goal region
     */
    int next_path_index;

    /**
     * flag to indicate whether the next vertex needs to be updated (if false) else 
     * continue moving on the edge between current_path_index and next_path_index
     */
    bool step_path;

    /**
     * size of the steps for moving on the PRM edges.
     */
    double step_size_path;

    /**
     * slope of the current edge being traversed either while moving on the PRM or 
     * while transiting on to the PRM
     */
    double current_path_slope;

    /**
     * populates the prm_edges[i] with neighbours of the ith vertex of the PRM
     * a neighbour is added to the list at prm_edges[i] if there exists a simple
     * local path between the ith vertex and the neighbour under consideration
     */
    void GetNeighboursByDirectInterpolation(const int i);

    /**
     * checks whether there exists a simple path between the points (start_x, start_y) 
     * and (end_x, end_y) in the given environment
     */
    bool IsLocalPathSimple(const double start_x, const double start_y, const double end_x, const double end_y);

    /**
     * returns the index of the closest vertex on the PRM based on the Euclidean distance
     * from the point (x,y)
     * returns -1 if no vertex found
     */
    int GetNearestNeighbourByDirectInterpolation(const double x, const double y);

    /**
     * This method uses Dijkstra's algorithm on the PRM constructed
     * to get the weights for all the vertices on the PRM using the 'start'
     * for the source index. This also sets the parents for all the vertices
     * using Dijkstra's algorithm itself.
     */
    void GetWeightsByDijkstra(const int start);

    /**
     * This method returns the index fo teh optimal vertex on the PRM
     * found using the sum of the weight assigned by Dijkstra's and the 
     * Euclidean distance of the vertex from the point (x,y)
     */
    int GetOptimumPRMVertex(const double x, const double y);

    /** 
     * This is called to reset and set the environment in a state for running the motion planner again.
     * This clears the entire PRM with its vertices and edges and weights and so on.
     */
    void ResetPRM();

    friend class Graphics;    
};

#endif
