#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include<math.h>

void DrawColor(const double r, const double g, const double b)
{
    glColor3d(r, g, b);    
}

void DrawPoint2D(const double x, const double y)
{
    glPointSize(4);    
    glBegin(GL_POINTS);
    glVertex2d(x, y);    
    glEnd();
    glPointSize(1);
}

void DrawSegment2D(const double x1, const double y1,
		   const double x2, const double y2)
{
    glLineWidth(3);    
    glBegin(GL_LINES);
    glVertex2d(x1, y1);
    glVertex2d(x2, y2);    
    glEnd();
    glLineWidth(1);
}

void DrawString2D(const double x, const double y, const char str[])
{
    if(str)
    {
	glRasterPos2d(x, y);
	for(int i = 0; str[i] != '\0'; ++i)
	    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }    
}

void DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	    glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}

Graphics *m_graphics = NULL;

Graphics::Graphics(MotionPlanner * const motionPlanner) 
{
    m_motionPlanner          = motionPlanner;
    m_motionPlannerDraw      = true;
    m_motionPlannerPRMSet    = false;
}

Graphics::~Graphics(void)
{
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Awesome Algorithm!!");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutIdleFunc(NULL);
    glutTimerFunc(0, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);
    glutSpecialFunc(CallbackEventOnSpecialKeyPress);

//create menu
    glutCreateMenu(CallbackEventOnMenu);

    MENU_RUN_MOTION_PLANNER          = 1;
    MENU_RESET_MOTION_PLANNER        = 2;
    MENU_SET_ROBOT_POSITION          = 3;
    MENU_SET_GOAL_POSITION           = 4;  
    MENU_DRAW_MOTION_PLANNER         = 5;
 
    glutAddMenuEntry("Run motion planner", MENU_RUN_MOTION_PLANNER);
    glutAddMenuEntry("Reset the motion planner", MENU_RESET_MOTION_PLANNER);
    glutAddMenuEntry("Set the Robot position", MENU_SET_ROBOT_POSITION);
    glutAddMenuEntry("Set the goal position", MENU_SET_GOAL_POSITION);
    glutAddMenuEntry("Draw motion planner [yes/no]", MENU_DRAW_MOTION_PLANNER);

    glutAttachMenu(GLUT_RIGHT_BUTTON);	

//enter main event loop
    glutMainLoop();	   
}

void Graphics::HandleEventOnTimer(void)
{
    /*Clock clk;
    ClockStart(&clk);*/
	m_motionPlanner->m_simulator->UpdateMovingObstacles();

    if(!m_motionPlanner->m_simulator->IsRobotCenterInsideGoalRegion() && m_motionPlannerPRMSet){
        m_motionPlanner->moveRobot();
    }
    //printf("Time elapsed: %f\n", ClockElapsed(&clk));
} 

void Graphics::HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY)
{	
    if (m_menuSelItem == MENU_SET_ROBOT_POSITION){
        m_motionPlanner->m_simulator->SetRobotPosition(mousePosX, mousePosY);
    }
    else if (m_menuSelItem == MENU_SET_GOAL_POSITION){
        m_motionPlanner->m_simulator->SetGoalPosition(mousePosX, mousePosY);
    }
    else return;

    m_menuSelItem = -1;
}

void Graphics::HandleEventOnMenu(const int item)
{
    m_menuSelItem = item;

    if(item == MENU_RUN_MOTION_PLANNER)
    {
        if (!m_motionPlannerPRMSet){
            m_motionPlannerPRMSet = m_motionPlanner->Solve();
        }
        else{
            printf("Please reset the motion planner using the reset option in the menu.\n");
        }
        if (!m_motionPlannerPRMSet){
            printf("Please re-run the motion planner as not the sample points could not be connected.\n");
        }
    }
    else if(item == MENU_DRAW_MOTION_PLANNER)
    {
	    m_motionPlannerDraw = !m_motionPlannerDraw;
	    printf("DRAW_MOTION_PLANNER: %s\n", m_motionPlannerDraw ? "YES" : "NO");	
    } 
    else if(item == MENU_RESET_MOTION_PLANNER)
    {
        m_motionPlanner->ResetPRM();
        m_motionPlannerPRMSet = false;
    }
    else if(item == MENU_SET_ROBOT_POSITION)
    {
	    printf("Left Click on the window to set robot position\n");
    }    
    else if(item == MENU_SET_GOAL_POSITION)
    {
	    printf("Left Click on the window to set goal position\n");
    }
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    switch(key)
    {
        case 27: //escape key
	        exit(0);
	
        case GLUT_KEY_F1: 
	        HandleEventOnHelp();	
	        break;
    }
}

void Graphics::HandleEventOnHelp(void)
{
    printf("Help: right click to display menu\n");
}


void Graphics::HandleEventOnDisplay(void)
{
    char str[100];
  
    if(m_motionPlannerDraw) m_motionPlanner->Draw();

//draw robot
    glColor3f(0.1016, 0.2656, 0.6445);
    DrawCircle2D(m_motionPlanner->m_simulator->m_robotCenterX, m_motionPlanner->m_simulator->m_robotCenterY, 
                 m_motionPlanner->m_simulator->m_robotRadius);

//draw goal
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(0.0742, 0.8086, 0.0938);
    DrawCircle2D(m_motionPlanner->m_simulator->m_goalCenterX, m_motionPlanner->m_simulator->m_goalCenterY, 
                 m_motionPlanner->m_simulator->m_goalRadius);
    
//draw static obstacles
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    glColor3i(0.5977, 0.3711, 0.7305);
    glBegin(GL_TRIANGLES);

    const int nStaticObstacles = m_motionPlanner->m_simulator->GetNrStaticObstacles();   
    for(int i = 0; i < nStaticObstacles; i++)
    {
        Simulator::StaticObstacle *staticObstacle = m_motionPlanner->m_simulator->m_static_obstacles[i];
	    const int ntri = staticObstacle->m_triangles.size();
    
	    for(int j = 0; j < ntri; j += 3)
	    {
	        glVertex2dv(&staticObstacle->m_vertices[2 * staticObstacle->m_triangles[j + 0]]);
	        glVertex2dv(&staticObstacle->m_vertices[2 * staticObstacle->m_triangles[j + 1]]);
	        glVertex2dv(&staticObstacle->m_vertices[2 * staticObstacle->m_triangles[j + 2]]);
	    }
    }
    glEnd();

//draw moving obstacles
    for(int i = 0; i < m_motionPlanner->m_simulator->GetNrMovingObstacles(); i++)
    {
        glColor3f(1, 0, 0);
        DrawCircle2D(m_motionPlanner->m_simulator->m_moving_obstacles[i]->centerX, m_motionPlanner->m_simulator->m_moving_obstacles[i]->centerY, 
                     m_motionPlanner->m_simulator->m_moving_obstacles[i]->radius);
        glColor3f(0, 0, 0);
        char str[100];
        sprintf(str, "%d", i);
        DrawString2D(m_motionPlanner->m_simulator->m_moving_obstacles[i]->centerX, m_motionPlanner->m_simulator->m_moving_obstacles[i]->centerY, str);
    }

}

void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	    glClearDepth(1.0);
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
    //	glEnable(GL_DEPTH_TEST);
    //	glShadeModel(GL_SMOOTH);	
	
	    glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	    glMatrixMode(GL_PROJECTION);
	    glLoadIdentity();

	    const double x = m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[0];
	
	    glOrtho(m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[0], 
		    m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMax()[0], 
		    m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[1], 
		    m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMax()[1], -1.0, 1.0);
	    glMatrixMode(GL_MODELVIEW);
	    glLoadIdentity();	    
	
	    m_graphics->HandleEventOnDisplay();
	
	    glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
	    double mouseX, mouseY;
	    MousePosition(x, y, &mouseX, &mouseY);
	    m_graphics->HandleEventOnMouseLeftBtnDown(mouseX , mouseY);
	    glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	    m_graphics->HandleEventOnTimer();
	    glutTimerFunc(0, CallbackEventOnTimer, id);
	    glutPostRedisplay();	    
    }
}

void Graphics::CallbackEventOnMenu(int item)
{
    if(m_graphics)
    {
	    m_graphics->HandleEventOnMenu(item);
	    glutPostRedisplay();
    }    
}

void Graphics::CallbackEventOnSpecialKeyPress(int key, int x, int y)
{
    if(m_graphics)
	    m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	    m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv)
{
    PseudoRandomSeed();
	
    if(argc < 2)
    {
	    printf("missing arguments\n");		
        printf("  MotionPlanner <scene.txt> [OPTIONAL] <n>\n");
	    printf("where\n");
	    printf("  <scene.txt> is one of the provided scene files\n");
	    printf("  <n> is the number of moving obstacles to be added in the scene : DEFAULT = 3\n");
	    return 0;	
    }

    int n;
    if (argv[2] != '\0')
        n = atoi(argv[2]);
    else
        n = DEFAULT_NR_MOVING_OBSTACLES;

    Simulator sim(argv[1], n);
    MotionPlanner mp(&sim);    
    Graphics graphics(&mp);
    
    graphics.HandleEventOnHelp();
    graphics.MainLoop();
    
    return 0;    
}
