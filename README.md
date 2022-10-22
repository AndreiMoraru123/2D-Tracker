# ObjectTracking
Linear, extended and unscented Kalman filter models for tracking the movement of an object in 2D interactive game in MATLAB

## Linear Kalman filter estimator for a linear trajectory:
Lqr/Place were used alternatively to actuate the shark's movement based on the estimations 


![lkf](https://user-images.githubusercontent.com/81184255/179504407-11330108-6403-45d3-b3c5-dfb5a9cc735d.gif)

<img src="https://user-images.githubusercontent.com/81184255/179504407-11330108-6403-45d3-b3c5-dfb5a9cc735d.gif" width="600" height="250"/>

### Performances: 

![image](https://user-images.githubusercontent.com/81184255/179503846-05b4d593-51a2-436c-98bc-dd6b6af85c88.png)

![image](https://user-images.githubusercontent.com/81184255/179503891-f7fc30a7-4693-4df2-b92d-ecbdb5cace05.png)


## Extended Kalman filter for a non linear trajectory with the help of a polar coordinates Seagull-sensor

![ekf](https://user-images.githubusercontent.com/81184255/179504427-cc6f5939-fa04-4080-9bfa-3db62bc611ab.gif)

### Performances: 

![image](https://user-images.githubusercontent.com/81184255/179504843-6e0cc412-f72b-492e-80b9-5cf73b9396ee.png)

![image](https://user-images.githubusercontent.com/81184255/179504868-80248a3e-bed6-4dbf-b2a3-17997683939a.png)


### EKF may diverge due to wrong derivative approximation:

![div](https://user-images.githubusercontent.com/81184255/179504661-1c1b513a-3f33-4f23-9dff-e86f4d63f3b3.gif)


## The Unscented Kalman filter for the most non linear random movement scenario proves to be a better solution:

![ukf](https://user-images.githubusercontent.com/81184255/179505178-7f32fcec-e6ec-4733-8cf6-e39c13a4b20b.gif)

### Yet at a higher cost:

![image](https://user-images.githubusercontent.com/81184255/179505243-8ac327ce-0883-43a6-8bb7-53b349e5cd03.png)

![image](https://user-images.githubusercontent.com/81184255/179505261-c53bde8e-b01c-4662-8f11-44aba3ce3f2b.png)




