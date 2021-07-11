## state-space model
### state equation

The state equation is described like below:

![state_equation_0.png](images/state_equation_0.png)
where **wk** is the process noise.

Assuming that the ω is small, the following model approximates the robot's motion.

![state_model.png](images/state_model.png)

![state_equation_1.png](images/state_equation_1.png)

When **⊿t** is defined as the tiem difference from k-1 to k, **⊿L** and **⊿θ** is defined like below:

![state_equation_2.png](images/state_equation_2.png)

So *f* is defined like below:

![state_equation_3.png](images/state_equation_3.png)

### observation equation

The observation equation is described like below:

![observation_equation_0.png](images/observation_equation_0.png)
where **vk** is the observation noise.

![observation_model.png](images/observation_model.png)

![observation_equation_1.png](images/observation_equation_1.png)

