# 3D Control for Quadcopter

##Project Instruction

In the Udacity Flying Car Nanodegree Lesson 12-14, I learned about the basic vehicle control, the control architecture and a full 3D cascaded control. In this project, I've learned how to implement the 3D control in C++ for a real quadcopter. The development setup and the project instruction are provided [here](./Instruction.md).

A quadcopter in 3D space has 12 state parameters:

![state](http://latex.codecogs.com/gif.latex?X%20%3D%20%5Cbegin%7Bbmatrix%7D%20x%2C%20y%2C%20z%2C%20%5Cphi%20%2C%20%5Ctheta%20%2C%20%5Cpsi%2C%20%5Cdot%7Bx%7D%2C%20%5Cdot%7By%7D%2C%20%5Cdot%7Bz%7D%2C%20p%2C%20q%2C%20r%20%5C%5C%20%5Cend%7Bbmatrix%7D)

As described in the lesson, the 3D Control architecture for a quadcopter can be illustrated as the below image:

![3D Control Architecture][control_arch]

## Implementation Details

The Udacity team has provided a simulator with 6 scenarios, where I can test my control function. 

### Scenario 1

The scenario 1 is the test scenario to see if all development setup is correctly installed and configured. With the default implementation of 

```
QuadControlParams.Mass * 9.81 / 4
```

I can change the correct drone mass to 0.5 kg.

## Scenario 2

As described in the lesson, I have to implement the body-rate-control (inner loop block) first and tune the parameter. Once I can rely the parameters, I can continue to implement the roll-pitch-controller. In order to test both control in the simulator, the function `GenerateMotorCommands(float collThrustCmd, V3F momentCmd)` has to be implemented correctly. Based on the given commanded collective thrust `collThrustCmd` and the commanded moment `momentCmd`, I need to solve these four linear equations:

    f_total =  f0 + f1 + f2 + f3
    f_rot_x =  f0 - f1 + f2 - f3
    f_rot_y =  f0 + f1 - f2 - f3
    f_rot_z = -f0 + f1 + f2 - f3

where 

* `f0` is the thrust of the up-left propeller with clockwise rotation,
* `f1` is the thrust of the up-right propeller with counterclockwise rotation,
* `f2` is the thrust of the down-left propeller with counterclockwise rotation,
* `f3` is the thrust of the down-right propeller with clockwise rotation,

which can be illustrated as following:

![rollpitch][rollpitch]
![pitchyaw][pitchyaw]

Since the rotation thrust in the z-axis is in the negative direction, I multiplied the above equation of `f_rot_z` with -1.

The rotation thrusts can be calculated using this equations:

    float f_rot_x = momentCmd.x / l; // 
    float f_rot_y = momentCmd.y / l; // 
    float f_rot_z = momentCmd.z / kappa;

and 

    float l = L / sqrt(2);         
    float f_total = collThrustCmd;

Before I return the thrust of each propeller, the `CONSTRAIN` function is call to constrain the thrust values like this:

    cmd.desiredThrustsN[0] = CONSTRAIN( f0, minMotorThrust, maxMotorThrust);


The result of the implementation can be seen in scenario 2 as below animation:

![scenario2][scenario2]

## Scenario 3

The next step is to implement the lateral-position-control, the altitude-control, and the yaw-control sequentially. Then, I tuned their gains. One important thing in the lateral-position-control is that the input parameters from the trajectory point (`curTrajPoint.velocity` and `curTrajPoint.accel` have to be modified. I implemented PD-with-feedforward control in this lateral-position-control block based on this quadcopter dynamics taken from [the section 3 of this paper](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf):

![quad dynamic](http://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20%5Cddot%7Bx%7D%5C%5C%20%5Cddot%7By%7D%5C%5C%20%5Cddot%7Bz%7D%20%5Cend%7Bbmatrix%7D%20%3D%20R%28t%29%20*%20%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%20c%28t%29%20%5Cend%7Bbmatrix%7D%20-%20%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%20g%20%5Cend%7Bbmatrix%7D%20%5CLeftrightarrow%20%5Cbegin%7Bmatrix%7D%20%5Cddot%7Bx%7D%20%3D%20c%28t%29%20b%5E%7Bx%7D%5C%5C%20%5Cddot%7By%7D%20%3D%20c%28t%29%20b%5E%7By%7D%20%5C%5C%20%5Cddot%7Bz%7D%20%3D%20c%28t%29%20b%5E%7Bz%7D%20-g%20%5Cend%7Bmatrix%7D)


In the altitude-control block, I implement the PID-with-feedforward. The integral term is integrated after observing the quad1 in the scenario4. For the yaw-control block, I implement the P control.

![scenario3][scenario3]

## Scenario 4

In this scenario, the parameter tuning of the integral controller kiPosZ was not trivial. I need to increase the speed of the lateral position by increasing the `kpPosXY` and the `kpVelXY`, in combination with the integral gain `kiPosZ`. Lower lateral gain and high integral gain will overshoot the quad1 before approaching the target position.

![scenario4][scenario4]

## Scenario 5 and 6

The scenario 5 and 6 are the extra challenge scenarios with more advance trajectories. Currently, I cannot pass all of the challenges. In the scenario 5, I still cannot suppress the error of the quad1:

![scenario5][scenario5]

In the scenario 6 with 9 quadcopters, I can see that 2 vehicles are crashed and other 7 vehicles can follow the trajectory.

![scenario6][scenario6]

[//]: # (References)
[control_arch]: ./animations/ControlArchitecture.jpg
[rollpitch]: ./animations/RollPitch.png
[pitchyaw]: ./animations/PitchYaw.png
[scenario1]: ./animations/yscenario1.gif
[scenario2]: ./animations/yscenario2.gif
[scenario3]: ./animations/yscenario3.gif
[scenario4]: ./animations/yscenario4.gif
[scenario5]: ./animations/yscenario5.gif
[scenario6]: ./animations/yscenario6.gif
