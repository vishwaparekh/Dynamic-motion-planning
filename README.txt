On Linux:

        To compile the source code on linux
        Run cmake . <path of CMakeLists file>
        e.g. if you are on the same path,
        run cmake .
        Run make 
        To run the program
        ./bin/MotionPlanner <path of the scene file> <number of moving obstacles required>

On Windows:
        Download and install CMake 2.6 or later.
        Give the src folder in the code .zip file as the source path in CMake.
        Give a build path and choose your development environment accordingly.
        The build path now contains the project files which can be compiled using a reliable compiler.
        
        Run the MotionPlanner.exe file created along with command line arguments of <scene filename> and <no. of moving obstacles>

Scenes: scene_w.txt, scene_wen.txt, scene_wenliu.txt

After a scene is loaded:
Right click anywhere on the GUI
You will have the following options
1. Run the motion planning algorithm: Will run the motion planning and also start moving the robot towards the goal.
2. Reset the Motion Planner: Reset all the parameters for running motion planning.
3. Change the position of robot: After clicking on this option, the next point you click on the GUI will be the new position
4. Change the position of goal: same
5. Animate the path [Yes/No]: Every click will alter the selection.