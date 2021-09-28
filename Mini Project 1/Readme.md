Although the code is admittedly incredibly messy, running it is fairly simple. It is entirely self-contained in one file and requires no inputs unless one wishes to change parameters from the given. It is tailored rather heavily to the project, having many sections repeat code for the 4 different path segments. If one wishes to change the paths, they must simply change the Part 3b named P0, P1, P2, P3, P4, theta0 and thetaf. These values determine the starting and ending positions and angles for the various segments. Theta variables are repeated for two segments so must be changed for each. By simply changing these, the paths generated will be changed. Similarly, by changing umax, wmax, amax and armax, you can change the kinematic models as you'll be changing the input velocity and acceleration maximums. Finally, one other key input group is the step sizes for the paths. N is for the linear segments, and n_p, n_a and n_s deal with the splnes. Change these will allow the user to change the size of arrays generated to track movement for the segments. Currently they are set to 20, 10, 8 and 50, respectively. Also, when there is more than one room frame plot to generate (i.e. the robot is following a path) it will prompt the user if they would like to save the movie to an avi file in the directory. Simply type Y to do so.