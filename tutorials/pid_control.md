# Introduction to control
For a more in-depth exploration of this topic, take ME 333 - introduction to mechatronics, ME 449 - robotic manipulation, EE 360 - introduction to feedback system, EE 374 - introduction to digital control, or CS 410 - quadrotor design and control
## Key concepts for today
- Control systems
- Open loop control
- Closed loop control
- PID control
## Control systems
This is somewhat self explanatory. A control system directs the behavior of other devices or systems. Control systems can range from extremely simple to extremely complex. For the sake of simplicity, we will just look at some basic control systems/theory at a very high level for now.
## Open loop control
This is the most basic control system and what you probably think of when it comes to control. Open loop control is when you just tell your system "do this" and you just blindly let it go. More formally, the action the controller takes is independent of its output. An example of an open loop controller is heating water to a set temperature by turning on a boiler for a set period of time. This system acts independently of what the water temperature actually ends up as, so while simple, there is considerable margin for error. For a robotics example, say we want to turn an underwater ROV 90 degrees to the right. In an open loop control system, we will simply drive the thrusters a set amount without paying attention to how much the robot actually turned.
## Closed loop control
This is a more robust system where the controller takes into account its output when performing an action. This is also known as feedback control. For the heating water example, we can feed the current temperature of the water into the controller and adjust the heating time based on that. In the UROV example, we can account for how far the robot actually turned and then adjust the thruster control from there.
## PID control
This is probably one of the most famous and also basic forms of feedback control. PID stands for proportional, integral, derivative. You can see why when we formally write out the control equation.

$$
u(t)= K_pe(t)+K_i\int e(t)dt+K_d\frac{d}{dt}e(t)
$$

where $u(t)$ is the controller's response, $e(t)$ is the system's error with respect to time, and $K_p$, $K_i$ and $K_d$ are the controller's proportional, integral, and derivative gains, respectively.

Let's break this down a little further. The most basic feedback controller we can work with is a P controller (basically $K_i=K_d=0$). Mathematically, it looks like this

$$u(t)=K_pe(t)$$

Essentially, we drive our system with some gain propertional to its error to correct for it. Going back to our robot example, if our desired angle is 90 degrees to the right, but we are only at 50 degrees right now, we have an error of 40 degrees. So we will drive our thrusters proportional to this error.

One notable quirk of P control is overshoot. Your control system will at some point overshoot, which will result in error in the opposite direction that the P controller will try to correct, often resulting it overshooting the target again, but this time on the other side. This behavior is called "ringing" and happens with pure P controllers.

To reduce ringing, we can add some damping to our system. We do this by adding the time derivative of the error to our control equation

$$u(t)=K_pe(t)+K_d\frac{d}{dt}e(t)$$

Now, the main issue with our controller is that there is some potential for some steady state error that we can't really get rid of. To fix this, we can integrate the error over time which mathematically takes us back to our first PID control equation.

$$
u(t)= K_pe(t)+K_i\int e(t)dt+K_d\frac{d}{dt}e(t)
$$

Note that this isn't a flawless controller. Integral control, especially if $K_i$ is too large, can add instability to your controller, which ends up being way worse than having some mild steady state error.