# Can you outrun the Big Bad ***Kalman filter*** ?

Some linear, extended and unscented movement tracking models, with a fun twist

## ___Noob level___: defeat the linear Kalman filter
#### The ___Shark___ can only chase you in a linear fashion

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504407-11330108-6403-45d3-b3c5-dfb5a9cc735d.gif" width="1000"/>
</p>

### Test each of your runs:

![image](https://user-images.githubusercontent.com/81184255/179503846-05b4d593-51a2-436c-98bc-dd6b6af85c88.png)

![image](https://user-images.githubusercontent.com/81184255/179503891-f7fc30a7-4693-4df2-b92d-ecbdb5cace05.png)

## Experienced: defeat the extended Kalman filter
#### The shark is getting help from a ___Seagull___ whose eyesight acts like an active visual sensor for detecting your non-linear movements

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504427-cc6f5939-fa04-4080-9bfa-3db62bc611ab.gif" width="1000"/>
</p>

### Measure your performances:

![image](https://user-images.githubusercontent.com/81184255/179504843-6e0cc412-f72b-492e-80b9-5cf73b9396ee.png)

![image](https://user-images.githubusercontent.com/81184255/179504868-80248a3e-bed6-4dbf-b2a3-17997683939a.png)

### Pro tip: You can trick the shark by moving fast in a non-linear manner (so the filter will diverge due to wrong partial derivatives calculation)

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179504661-1c1b513a-3f33-4f23-9dff-e86f4d63f3b3.gif" width="1000"/>
</p>

## Legendary: defeat the unscented Kalman filter (and enter Valhalla (probably, I don't make the rules))

#### No more linear covariance transforms, the ___Shark___ has unlocked the ___Unscented Transform___ ability

<p align="center">
  <img src="https://user-images.githubusercontent.com/81184255/179505178-7f32fcec-e6ec-4733-8cf6-e39c13a4b20b.gif" width="1000"/>
</p>

### And see how far your can get:

![image](https://user-images.githubusercontent.com/81184255/179505243-8ac327ce-0883-43a6-8bb7-53b349e5cd03.png)

![image](https://user-images.githubusercontent.com/81184255/179505261-c53bde8e-b01c-4662-8f11-44aba3ce3f2b.png)

## Yes, ___you___ control the ___Seal___, [this is how](https://github.com/AndreiMoraru123/SensorFusion)

### How this ___madness___ was designed:

![image](https://user-images.githubusercontent.com/81184255/197408389-ee578ee3-afc0-4a37-9849-c39d1b0e351b.png)

### and engineered:

![image](https://user-images.githubusercontent.com/81184255/197408420-c3ce43d4-9f16-4144-a9f6-a521eda4e074.png)

### and if you made it this far...

#### here is the whole thing explained in detail (Vampire language):

[OneFilterToRuleThemAll.pdf](https://github.com/AndreiMoraru123/ObjectTracking/files/9847220/OneFilterToRuleThemAll.pdf)



