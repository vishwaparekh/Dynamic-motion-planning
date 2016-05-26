/**
 *@file Simulator.hpp
 *@author Erion Plaku 
 *@brief Problem information
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include "Utils.hpp"

#define DEFAULT_NR_MOVING_OBSTACLES 3
#define NR_LASER_SCAN_STEPS 20
#define NR_LASER_SCAN_ANGLES 20
#define MAX_ROBOT_RADIUS 0.4
#define MIN_ROBOT_RADIUS 0.2
#define MAX_MOVING_OBS_RADIUS 1.0
#define MIN_MOVING_OBS_RADIUS 0.5

/**
 *@author Anand and Vishwa
 *@remark
 * Input
 * - m circular moving obstacles M_1, M_2, ..., M_m
 *   their motion is unknown
 * - n polygonal static obstacles S_1, S_2, ..., S_n
 *   their vertices are known before hand
 * - one circular robot that can translate in 2D space depending on heading direction
 *   with a laser sensor to get hits at certain regular intervals of angles
 * - one circular goal region which is the destination of the motion planner
 * 
 * Objective
 * - reach the goal region avoiding any obstacles on the way using the laser sensor
 *
 *@remark
 *  Simulator provides functionality to access information about the obstacles, 
 *  the goal regions, and the robot. The simulator also allows the user to 
 *  - set the current position of the robot
 *  - get laser sensor readings
 *  - check whether the robot in its current placement is in collision with the obstacles
 *  - check whether the robot center in its current placement is inside the goal region
 *  .
 */

class Simulator
{
public:    
    /**
     *@brief Initialize variables
     *@param fname name of file with obstacles and reward regions
     */
    Simulator(const char fname[]);

    /**
     *Constructor for the simulator using the scene file and number of moving obstacles.
     */
    Simulator(const char fname[], const int n);
    
    /**
     *@brief Delete allocated memory
     */
    ~Simulator(void);

     /**
     * This is used to set the robot at a randomly selected position
     * after chacking for collision with the goal region and the static obstacles.
     */
    void SetupRobot();

    /**
     * This is used to set the goal at a randomly selected position
     * after checking for collision with the static obstacles.
     */
    void SetupGoal();

/**
 *@name Information about the obstacles
 *@{
 */
    /**
     *returns the number of static obstacles in the scene.
     */ 	
    int GetNrStaticObstacles(void) const {return m_static_obstacles.size();}

    /**
     * returns the number of movin obstacles defined in the scene
     */
    int GetNrMovingObstacles(void) const {return nr_moving_obstacles;}

    /**
     * Get number of vertices of the i-th static obstacle
     */ 	
    int GetObstacleNrVertices(const int i) const {return m_static_obstacles[i]->m_vertices.size() / 2;}

    /**
     *@brief Get vertices of the i-th obstacle
     *@remark
     *  Each obstacle is represented as a polygon.
     *  You can obtain the vertices of the i-th obstacle
     *  by calling this function as follows
     *  <CENTER>
     *   const double *vertices = m_simulator->GetObstacleVertices(i);
     *  </CENTER>
     *  Then, the x-coordinate of the j-th vertex is given by
     *  <CENTER>
     *         vertices[2 * j]
     *  </CENTER>
     *  and the y-coordinate of the j-th vertex is given by
     *  <CENTER>
     *         vertices[2 * j + 1]
     *  </CENTER>
     * 
     */ 	
    const double* GetObstacleVertices(const int i) const
    {
	    return &(m_static_obstacles[i]->m_vertices[0]);
    }

    /**
     * logic for the random motion of the moving obstacles taking into account
     * the other moving obstacles and static obstacles and changing directions
     * according to their respective changeTheta parameters.
     */
    void UpdateMovingObstacles();

/**
 *@name Information about the goal
 *@{
 */
    /**
     *@brief Get goal's radius
     */ 	
    double GetGoalRadius(void) const {return m_goalRadius;}
    
    /**
     *@brief Get x-coordinate of goal's center
     */ 	
    double GetGoalCenterX(void) const {return m_goalCenterX;}
    
    /**
     *@brief Get y-coordinate of goal's center
     */ 	
    double GetGoalCenterY(void) const {return m_goalCenterY;}

    /**
     * Sets the goal position to the passed parameters.
     */
    void Simulator::SetGoalPosition(const double x, const double y) {
        m_goalCenterX = x;
        m_goalCenterY = y;
    }

    /**
     * Get robot radius (robot is a circle)
     */ 	
    double GetRobotRadius(void) const {return m_robotRadius;}
    
    /**
     * Get x-coordinate of robot's center
     */ 	
    double GetRobotCenterX(void) const {return m_robotCenterX;}
    
    /**
     * Get y-coordinate of robot's center
     */ 	
    double GetRobotCenterY(void) const {return m_robotCenterY;}
    
    /**
     *@brief Set robot orientation and position
     *@param x new x-coordinate of robot's center
     *@param y new y-coordinate of robot's center
     */ 	
    void Simulator::SetRobotPosition(const double x, const double y) {
        m_robotCenterX = x;
        m_robotCenterY = y;
    }
    
    /**
     *@brief Return true iff the robot in its current placement is in collision with an obstacle
     */
    bool IsRobotInCollision(void) const;

    /**
     *@brief Return true iff the robot center is inside the goal region
     */
    bool IsRobotCenterInsideGoalRegion() const;

    /**
     *@brief Return true iff the point (x, y) is inside the goal region
     *@param x x-coordinate of point
     *@param y y-coordinate of point
     */
    bool IsPointInsideGoalRegion(const double x, const double y) const;
    
    /**
     * This function scans the environment using the lasser sensor
     * returns
     *  true    a hit from a MOVIG OBSTACLE
     *  false   NO HIT or a hit with a STATIC OBSTACLE
     */
    bool ScanSurrounding();

    /**
     * This method uses the laser to scan at the specified angle theta 
     * and sets the hit point's position in xhit and yhit.
     * xhit and yhit are MAXINT in case of NO HIT
     * true - hit with a MOVING OBSTACLE
     * false - NO HIT or hit with a STATIC OBSTACLE
     */
    bool TakeStaticLaserReading(const double theta, double &xhit, double &yhit); 

/**
 *@name Information about the bounding box
 *@{
 */
    /**
     *@brief Get minimum (x, y) coordinates of bounding box
     */ 	
    const double* GetBoundingBoxMin(void) const {return m_min;}
    
    /**
     *@brief Get maximum (x, y) coordinates of bounding box
     */ 	
    const double* GetBoundingBoxMax(void) const {return m_max;}
    

protected:
    /**
     *@brief Read polygonal obstacles and reward regions from input file
     *
     *@param fname name of file with obstacles and reward regions
     */ 	
    void SetupFromFile(const char fname[]);

    /**
     *@brief Obstacles: each obstacle corresponds to a polygon
     */
    struct StaticObstacle
    {
	    std::vector<double> m_vertices;
	    std::vector<int> m_triangles;
    };

    /**
     * initializes the moving obstacles at random positions in the scene avoiding other static obstacles and the robot
     * their radii are also random in the range of [MIN_MOVING_OBS_RADIUS, MAX_MOVING_OBS_RADIUS]
     * their initial moving directions are set randomly also [theta]
     * their change direction values are set randomly too [changeTheta]
     */
    void SetupMovingObstacles();

    /**
     * sets the 'i'th moving obstacle's center at the point (x,y)
     */
    void SetMovingObstaclePosition(const int i, const double x, const double y);

    /**
     * changes the moving direction 'theta' of the 'i'th moving obstacle
     * according to its 'changeTheta' parameter
     */
    void ChangeMovingObstacleDirection(const int i);

    /**
     * structure to handle the moving obstacles
     * (centerX, centerY) is the center position
     * radius is the radius of the obstacle
     * speed is the step size for the obstacle
     * theta is the moving direction 
     * changeTheta is the change made in theta while encountering another obstacle
     */
    struct MovingObstacle
    {
	    double centerX;	
	    double centerY;
	    double radius;
        double speed;
	    double theta;
        double changeTheta;
    };

    /**
     *@brief All the obstacles
     */
    std::vector<StaticObstacle *>  m_static_obstacles;
    std::vector<MovingObstacle *>  m_moving_obstacles;
    int nr_moving_obstacles;

    /**
     * this method returns appropriately selected random (center_x,center_y)
     * pair by checking for collision with the static obstacles.
     */
    void SetupCircle(const double radius, double& center_x, double& center_y);

    /**
     *@brief Robot information: robot is a circualar body
     */
    double m_robotRadius;
    double m_robotCenterX;
    double m_robotCenterY;    

    /**
     *@brief Goal information: goal is a circular region
     */
    double m_goalRadius;
    double m_goalCenterX;
    double m_goalCenterY;

    /**
     *@brief Bounding box: minimum and maximum scene values
     */
    double m_min[2];
    double m_max[2];    

    friend class Graphics;
};

#endif
