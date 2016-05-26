#include "Simulator.hpp"
#include <iostream>
#include <math.h>

using namespace std;

/**
 * constructor for the Simulator class
 * fname : filename containing static obstacles
 */
Simulator::Simulator(const char fname[])
{
    Simulator(fname, DEFAULT_NR_MOVING_OBSTACLES);
}

/**
 * constructor overload with 
 * fname : filename containing static obstacles
 * n : number of moving obstacles to be created
 */
Simulator::Simulator(const char fname[], const int n)
{
//default values

//default minimum/maximum values of scene bounding box
    m_min[0] = -22;
    m_min[1] = -14;
    m_max[0] =  22;
    m_max[1] =  14;

// static obstacles setup from the scene file
    SetupFromFile(fname);

//default dimensions of the goal
    m_goalRadius = 0.5;

//default position and orientation of the goal
    SetupGoal();

//default dimensions of the robot
    m_robotRadius = PseudoRandomUniformReal(MIN_ROBOT_RADIUS, MAX_ROBOT_RADIUS);

//default position and orientation of the robot
    SetupRobot();

// set up moving obstacles randomly
    nr_moving_obstacles = n;
    SetupMovingObstacles();
}

Simulator::~Simulator(void)
{
//delete static obstacles
    const int ns = m_static_obstacles.size();
    for(int i = 0; i < ns; ++i){
	    if(m_static_obstacles[i]){
	        delete m_static_obstacles[i];
        }
    }
//delete moving obstacles
    for (int i = 0; i < nr_moving_obstacles; i++){
        if (m_moving_obstacles[i]){
            delete m_moving_obstacles[i];
        }
    }
}

void Simulator::SetMovingObstaclePosition(const int i, const double x, const double y)
{
    m_moving_obstacles[i]->centerX = x;
    m_moving_obstacles[i]->centerY = y;
}

void Simulator::ChangeMovingObstacleDirection(const int i)
{
    double theta = m_moving_obstacles[i]->theta;
    theta += m_moving_obstacles[i]->changeTheta;
    theta -= floor(theta / (2*M_PI)) * (2*M_PI); //maintain theta value between 0 and 2*PI
    m_moving_obstacles[i]->theta = theta;
}

void Simulator::UpdateMovingObstacles()
{
	for(int i = 0; i < nr_moving_obstacles; i++){
		bool inCollision = false; // to indicate if any collision can occur

        // move obstacle as per its speed in its heading direction theta
        double pt_x = m_moving_obstacles[i]->centerX + m_moving_obstacles[i]->speed * cos(m_moving_obstacles[i]->theta);
		double pt_y = m_moving_obstacles[i]->centerY + m_moving_obstacles[i]->speed * sin(m_moving_obstacles[i]->theta);
        double radius = m_moving_obstacles[i]->radius;
        
        // check for collision with all the static obstacles
        for(int j = 0; j < (int) m_static_obstacles.size(); ++j){
	    	const int nv = m_static_obstacles[j]->m_vertices.size() / 2;
	    	const double *poly = &(m_static_obstacles[j]->m_vertices[0]);
            if (CirclePolygonIntersection(pt_x, pt_y, radius, nv, poly)){
                inCollision = true;
                break;
            }
		}

        // on detection of collision change the heading direction
        if (inCollision){
            ChangeMovingObstacleDirection(i);
            continue;
        }

        // check for collision with other moving obstacles
        for (int j = 0; j < nr_moving_obstacles ; j++){
            if(j != i){
                if (CircleCircleIntersection(pt_x, pt_y, radius, m_moving_obstacles[j]->centerX, m_moving_obstacles[j]->centerY,
                    m_moving_obstacles[j]->radius)){
                        inCollision = true;
                        break;
                }
            }
        }

        // on detection of collision change the heading direction
        if (inCollision){
            ChangeMovingObstacleDirection(i);
            continue;
        }

        // check if the movign obstacle if headed outside the bounding box
        if (pt_x - radius < m_min[0] || pt_x + radius > m_max[0] || pt_y - radius < m_min[1] || pt_y + radius > m_max[1]){
            inCollision = true;
            ChangeMovingObstacleDirection(i);
            continue;
        }

        // if no collisions were detected then move the obstacle to the new position
        if (!inCollision){
            SetMovingObstaclePosition(i, pt_x, pt_y);
        }
	}
}

bool Simulator::IsRobotInCollision(void) const
{
    // check for collision with all the static obstacles
    for(int i = 0; i < (int) m_static_obstacles.size(); ++i)
    {
	    const int     nv   = m_static_obstacles[i]->m_vertices.size() / 2;
	    const double *poly = &(m_static_obstacles[i]->m_vertices[0]);
	
        if(CirclePolygonIntersection(m_robotCenterX, m_robotCenterY, m_robotRadius, nv, poly))
	        return true;
    }

    // check for collision with all moving obstacles
    for (int i = 0; i < nr_moving_obstacles; i++){
        if (CircleCircleIntersection(m_robotCenterX, m_robotCenterY, m_robotRadius, 
            m_moving_obstacles[i]->centerX, m_moving_obstacles[i]->centerY, m_moving_obstacles[i]->radius))
                return true;
    }

    return false;    
}

bool Simulator::IsRobotCenterInsideGoalRegion() const
{
    return IsPointInsideGoalRegion(m_robotCenterX, m_robotCenterY);
}

bool Simulator::IsPointInsideGoalRegion(const double x, const double y) const
{
    return IsPointInsideCircle(x, y, m_goalCenterX, m_goalCenterY, m_goalRadius);
}

bool Simulator::ScanSurrounding()
{
    //dummy variables
    double xhit, yhit;

    // scan in all directions in steps of
    for(int n = 0; n < NR_LASER_SCAN_ANGLES; n++)
	{
        double theta = 2*n*M_PI/NR_LASER_SCAN_ANGLES;

        // check if the laser sensor detected any hits
		if (TakeStaticLaserReading(theta, xhit, yhit))
            return true;
	}
    return false;
}

bool Simulator::TakeStaticLaserReading(const double theta, double &xhit, double &yhit)
{
    // get current robot position
	const double x1 = m_robotCenterX;
	const double y1 = m_robotCenterY;
	bool moving = false;

    // steps to scan the environments radially
	double step_size = 0.1;
	// initialize to deafult values returned in case of NO HIT
    xhit = MAXINT;
	yhit = MAXINT;
    // to indicate if any hits were encountered
	bool inCollision = false;

    for (int i = 0; i < NR_LASER_SCAN_STEPS; i++)
	{
        // get point in the theta direction at the current step
        double x2 = x1 + (i*step_size + m_robotRadius) * cos(theta);
        double y2 = y1 + (i*step_size + m_robotRadius) * sin(theta);

        // check for collision with the bounding box
        if (x2 > m_max[0] || x2 < m_min[0] || y2 > m_max[1] || y2 < m_min[1]){
            xhit = x2; yhit = y2;
            inCollision = true;
            break;
        }

        // check for collision with all the static obstacles
		for(int j = 0; j < m_static_obstacles.size(); j++)
		{
            const int nv = m_static_obstacles[j]->m_vertices.size() / 2;
	        const double *poly = &(m_static_obstacles[j]->m_vertices[0]);
            
            if(SegmentPolygonIntersection(x1, y1, x2, y2, nv, poly)) {
				xhit = x2; yhit = y2;
				inCollision = true;
				break;
			}
		}

        if(inCollision) break;

        // check for collision with all the moving obstacles
		for(int j = 0; j < m_moving_obstacles.size(); j++)
		{
            if(SegmentCircleIntersection(x1, y1, x2, y2, m_moving_obstacles[j]->centerX,
                m_moving_obstacles[j]->centerY, m_moving_obstacles[j]->radius)){
				    xhit = x2; yhit = y2;
				    inCollision = true;
				    moving = true;
				    break;
			}
		}
		if (inCollision) break;
	}

    return moving;
}


void Simulator::SetupFromFile(const char fname[])
{
    //file with obstacles
    //
    //file format is just a sequence of polygons

    FILE *in = fopen(fname, "r");
    if(in)
    {
	    int nrStaticObstacles;
	    int nrVertices;
        StaticObstacle *staticObstacle;
	
        // first line contains the number of static obstacles
	    if(fscanf(in, "%d", &nrStaticObstacles) != 1)
	    {
	        printf("error: expecting number of obstacles\n");
	        fclose(in);
	        return;
	    }

	    for(int i = 0; i < nrStaticObstacles; i++)
	    {
            // number of vertices for the current obstacle being read
	        if(fscanf(in, "%d", &nrVertices) != 1)
	        {
		        printf("error: expecting number of vertices for obstacle %d\n", i);
		        fclose(in);
		        return;
	        }
            staticObstacle = new StaticObstacle();

            staticObstacle->m_vertices.resize(2 * nrVertices);	    
            // list of vertices belonggin to the current obstacle
	        for(int i = 0; i < 2 * nrVertices; i++){
		        fscanf(in, "%lf", &(staticObstacle->m_vertices[i]));
            }

            // list of triangles to be used to draw the current obstacle using openGL
            staticObstacle->m_triangles.resize(3 * (nrVertices - 2));
	        for(int i = 0; i < (int) staticObstacle->m_triangles.size(); i++){
		        fscanf(in, "%d", &(staticObstacle->m_triangles[i]));
            }

            // add the static obstacle to the list of static obstacles
	        m_static_obstacles.push_back(staticObstacle);	    
	    }
	    fclose(in);
    }	
    else
	    printf("..could not open file <%s>\n", fname);
}

void Simulator::SetupCircle(const double radius, double& center_x, double& center_y)
{
    double x, y;
    bool inCollision = true;

    // until collision free points are found
    while (inCollision)
    {
	    inCollision = false;
        // sample center for the circle  of the given radius value in the available space
        x = PseudoRandomUniformReal(m_min[0] + radius, m_max[0] - radius);
        y = PseudoRandomUniformReal(m_min[1] + radius, m_max[1] - radius);

        // check for collision with all the static obstacles
	    for(int i = 0; i < (int) m_static_obstacles.size(); ++i)
        {
	        const int     nv   = m_static_obstacles[i]->m_vertices.size() / 2;
	        const double *poly = &(m_static_obstacles[i]->m_vertices[0]);
	
            // first check if the circle is in the interior of the obstacle
            if (IsPointInsidePolygon(x, y, nv, poly)){
                inCollision = true;
                break;
            }
            // check for circle intersection with the polygon edges
            if (CirclePolygonIntersection(x, y, radius, nv, poly)){
	            inCollision = true;
                break;
            }
        }
    }

    // set the center's position
    center_x = x;
    center_y = y;
}

void Simulator::SetupRobot()
{
    double x, y;    
    // gets collision free points until they are also away from the goal region
    do {
        SetupCircle(m_robotRadius, x, y);
    } while (IsPointInsideGoalRegion(x, y));
    SetRobotPosition(x, y);
}

void Simulator::SetupGoal()
{
    SetupCircle(m_goalRadius, m_goalCenterX, m_goalCenterY);
}

void Simulator::SetupMovingObstacles()
{
	for(int i = 0; i < nr_moving_obstacles; i++)
	{
        // randomly generate obstacle radius value
        double radius = PseudoRandomUniformReal(MIN_MOVING_OBS_RADIUS, MAX_MOVING_OBS_RADIUS);
        double pt_x, pt_y;
        bool inCollision = true;

        // loop until collision free points found
        while(inCollision)
        {
            inCollision = false;
            // sample points in the bounding box
            pt_x = PseudoRandomUniformReal(m_min[0] + 3, m_max[0] - 3);
            pt_y = PseudoRandomUniformReal(m_min[1] + 3, m_max[1] - 3);

            // check for collision with the robot
            if (CircleCircleIntersection(pt_x, pt_y, radius, m_robotCenterX, m_robotCenterY, m_robotRadius)){
                inCollision = true;
                continue;
            }

            // check for collision with all the static obstacles
            for(int j = 0; j < (int) m_static_obstacles.size(); ++j) {
	    		const int nv = m_static_obstacles[j]->m_vertices.size() / 2;
	    		const double *poly = &(m_static_obstacles[j]->m_vertices[0]);

                // check if the point is inside the static obstacle
                if (IsPointInsidePolygon(pt_x, pt_y, nv, poly)){
                    inCollision = true;
                    break;
                }

                // check if the moving obstacle is in intersection with the polygonal static obstacle's edges
                if (CirclePolygonIntersection(pt_x, pt_y, radius, nv, poly)){
                    inCollision = true;
                    break;
                }
		    }

            if (inCollision) continue;

            // check for collision with other existing moving obstacles
            for (int ii = 0; ii < i; ii++){
                if (CircleCircleIntersection(pt_x, pt_y, radius, m_moving_obstacles[ii]->centerX, m_moving_obstacles[ii]->centerY,
                    m_moving_obstacles[ii]->radius)){
                        inCollision = true;
                        break;
                }
            }

            if (inCollision) continue;
        }

        // create new moving obstacle as collision free center has been found
        MovingObstacle *movingObstacle;
		movingObstacle = new MovingObstacle();
		movingObstacle->centerX = pt_x;
		movingObstacle->centerY = pt_y;
		movingObstacle->radius = radius;
        movingObstacle->speed = PseudoRandomUniformReal(0.05, 0.1);
		movingObstacle->theta = PseudoRandomUniformReal(0, 2*M_PI);
        //movingObstacle->changeTheta = PseudoRandomUniformInteger(1, 3) * M_PI / 2;
        movingObstacle->changeTheta = PseudoRandomUniformReal(0, M_PI);
        //movingObstacle->changeTheta = M_PI/2;
		m_moving_obstacles.push_back(movingObstacle);
	}
}
