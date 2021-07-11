## pose estimation using EKF
### predict step

Jacobian of state equation is like below:

![state_jacobian.png](images/state_jacobian.png)

#### (1) predict pose

![ekf_1.png](images/ekf_1.png)

#### (2) predict covariance

![ekf_2.png](images/ekf_2.png)

Here **Q** is the covariance of the process noises (**wk**) which is assumed to be zero mean multivariate Gaussian noise.  
In this simulation, **Q** is an fixed matrix.

### update step

Jacobian of observation equation is like below:

![observation_jacobian.png](images/observation_jacobian.png)

#### (3) calculate measurement residual

![ekf_3.png](images/ekf_3.png)

#### (4) calculate residual covariance

![ekf_4.png](images/ekf_4.png)

Here **R** is the covariance of the observation noises (**vk**) which is assumed to be zero mean multivariate Gaussian noise.  
In this simulation, **R** is an fixed matrix.

#### (5) calculate Kalman Gain

![ekf_5.png](images/ekf_5.png)

#### (6) update pose

![ekf_6.png](images/ekf_6.png)

#### (7) update covariance

![ekf_7.png](images/ekf_7.png)

