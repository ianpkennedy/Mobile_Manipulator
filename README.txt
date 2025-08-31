ME449 Capstone Project - Ian Kennedy

This folder contains the implementation of the ME449 capstone project. In order to run the code for this directory, 
navigate to the /code directory, and place the modern_robotics python library in that directory. Block start and end 
configurations, along with control gains, can be changed in the main scope of the file. In order to run the script, enter the following in a terminal
window:
python3 Kennedy_Ian_capstone.py
Ensure that the ModernRobotics library is in the directory of the code. Ensure that the appropriate Kp and Ki gain matrices, along with the
appropriate cube start and end configurations are uncommented in the main scope of the python file.

The output of this program is the following:
1) A plot of the 6 positions of the error twist over the course of the trajectory
2) A .csv file of the error twist over time (xerr.csv)
3) A .csv file of the robot configuration parameters over time (Kennedy_Ian_capstone.csv)
4) The reference trajectory reference.csv

Joint collision and singularity avoidance was implemented in each of the three scenarios of the project. To achieve this, the robot
was started in a configuration where joint angles 2, 3 and 4 were below -0.25 rad. Then a restriction was placed on the joints such that 
when those aforementioned joint angles were above -0.25 rad, the corresponding Jacobian column was set to 0, preventing further 
movement in the undesired direction.  In each directory a "bad" video is also recorded showing the robot performing the task without joint limits. 
Keeping joints 2 through 4 below -0.25 keeps the arm "in front" of the chassis while also preventing the arm from 
collapsing in on itself.

The pseudoinverse operation for computing movement commands was also checked for entries close to 0 to prevent large joint 
commands. Values below 1e-3 were set to 0 because of this.

3 instances of the program are in the submission: /best, /overshoot, and /newTask

/newTask likely exhibits a very small amount of steady state error due to it not having an integrator form of control to eliminate steady state error.
/best also displays a very small amount of steady state error because it does not have integrator control. I also noticed that there was a slight
increase in error when imposing joint limits. This is likely due to the non linear way that joint limits were imposed in this implementation of
control. 
