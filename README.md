# Can you outrun the ***Big Bad Kalman filter*** ?

Some linear, extended and unscented movement tracking Kalman filters, with a fun twist

[![View Object Tracking via Sensor Fusion on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/119448-object-tracking-via-sensor-fusion)  

![image](https://user-images.githubusercontent.com/81184255/214925122-42760297-bee4-46b9-b61c-1654a0afe73a.png)

Run `ObjectTracker.m` and make sure all files are in the same directory. Set your scenarios using the dropdowns. Press `Play` and enjoy :-)

Go for `Developer Mode` (button) if you want to generate your custom data and play around with the trackers:

Model Parameters                    |           Filter Tuning             |         Extra Sensor
:-------------------------:|:-------------------------:|:-------------------------:
![1](https://user-images.githubusercontent.com/81184255/214925891-7a3f8fea-b96b-4f73-a41d-222dbc60d5d3.jpg) | ![2](https://user-images.githubusercontent.com/81184255/214925912-268b9881-238d-4b7b-843f-9e194c28a961.jpg) | ![3](https://user-images.githubusercontent.com/81184255/214925928-aac2f461-0552-4b74-bc5c-a002825dee9f.jpg)

> **Note** 
> You can control the Seal, if you own an Arduino + MPU inertial sensors suite, [this is how it works](https://github.com/AndreiMoraru123/SensorFusion).

> To achieve this, you may choose `Command Driven` instead of `Simulation` for the Running Mode.

# Demos

## ___Noob level___: defeat the linear Kalman filter
#### The ___Shark___ can only chase you in a linear fashion

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504407-11330108-6403-45d3-b3c5-dfb5a9cc735d.gif" width="700"/>
</p>

### Test each of your runs:

![image](https://user-images.githubusercontent.com/81184255/179503846-05b4d593-51a2-436c-98bc-dd6b6af85c88.png)

![image](https://user-images.githubusercontent.com/81184255/179503891-f7fc30a7-4693-4df2-b92d-ecbdb5cace05.png)

## ___Experienced___: defeat the extended Kalman filter
#### The shark is getting help from a ___Seagull___ whose eyesight acts like an active visual sensor for detecting your non-linear movements

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504427-cc6f5939-fa04-4080-9bfa-3db62bc611ab.gif" width="700"/>
</p>

### Measure your performances:

![image](https://user-images.githubusercontent.com/81184255/179504843-6e0cc412-f72b-492e-80b9-5cf73b9396ee.png)

![image](https://user-images.githubusercontent.com/81184255/179504868-80248a3e-bed6-4dbf-b2a3-17997683939a.png)

### Pro tip: You can trick the shark by moving fast in a non-linear manner (so the filter will diverge due to wrong partial derivatives calculation)

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504661-1c1b513a-3f33-4f23-9dff-e86f4d63f3b3.gif" width="700"/>
</p>

## ___Legendary___: defeat the unscented Kalman filter (and enter Valhalla (probably, I don't make the rules))

#### No more linear covariance transforms, the ___Shark___ has unlocked the ___Unscented Transform___ ability

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179505178-7f32fcec-e6ec-4733-8cf6-e39c13a4b20b.gif" width="700"/>
</p>

### And see how far your can get:

![image](https://user-images.githubusercontent.com/81184255/179505243-8ac327ce-0883-43a6-8bb7-53b349e5cd03.png)

![image](https://user-images.githubusercontent.com/81184255/179505261-c53bde8e-b01c-4662-8f11-44aba3ce3f2b.png)

### How this ___madness___ was designed:

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/197408389-ee578ee3-afc0-4a37-9849-c39d1b0e351b.png" width="700"/>
</p>

### engineered:

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/197408420-c3ce43d4-9f16-4144-a9f6-a521eda4e074.png" width="1000"/>
</p>

### and programmed:

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/204482601-fd1a1090-2fc8-4000-8904-ab36de3ed057.png" width="1000"/>
</p>

### and if you made it this far...

#### here is the whole thing explained in detail (Vampire language):

[OneFilterToRuleThemAll.pdf](https://github.com/AndreiMoraru123/ObjectTracking/files/9847220/OneFilterToRuleThemAll.pdf)



