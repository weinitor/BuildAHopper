# BuildAHopper
Step by step guide to building a hopper simulation in MATLAB.

## Bouncing Ball (simplified mechanics)
To begin, let's consider a basic scenario involving a bouncing ball. 
Our goal is to replicate the motion of an elastic ball as it drops from a specific height and bounces back up. 
The ball exhibits ballistic behavior during its flight, while the compression and release phases on the ground resemble a mass-spring-damper system.
Unfortunately, this is a complex problem that cannot be solved analytically and does not have a closed-form solution.

<p align="center">
<img src="figs/fig1.png" alt="Simplified mechanics of a bouncing ball" height="300"/>
</p>

We will drop a ball with a radius of $\chi_0=0.05(\text{m})$ and a weight of $1(\text{kg})$ from a height of $1(\text{m})$. 
The ball will have a total potential energy of $E$ at the start. 
In the absence of a damper, the ball will continue to bounce endlessly and will not lose any energy.
Thus we assume that the ground absorbs a certain amount of energy each time the ball hits it and only returns a fraction (elastic efficiency $\gamma=0.8$) of the previous energy $E_1$, resulting in $E_2=\gamma E_1$ as the new system energy after each collision. 
We will also assume that the collision is instantaneous and ignore any deformations during that time.

> Write out the total energy of the system $E$. Next, determine the coefficient of restitution $\psi$, or the ratio of the velocity before the collision (represented as $\dot{\chi}_2$) to the velocity after the collision (represented as $\dot{\chi}_1$). In other words, find $\psi=-\dot{\chi}_2/\dot{\chi}_1$.

> Now construct its equation of motion as an initial value problem with a set of state-space representation $\dot{\mathbf{x}}=f(\mathbf{x})$ (first order ODEs) having different assigned velocity after every collision, where $\mathbf{x}=\left[\chi,\dot{\chi}\right]^T$

> We will now solve the equation of motion numerically using ODE45 from MatLab and plot the solution with respect to time. Modify the coefficient of restitution (param.rho) in `SimplifyBouncingBall.m` to see the result.
