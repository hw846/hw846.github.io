---
layout: default
---
# Lab 5 Overview:
In this lab, I explore how to implement PID control in my developed robot system and understand how position control works.
```Final Wordcount: XXX``` (Extra due to Graduate Tasks)

#### Pre-Lab
For this pre-lab, the major objective was to setup a system for debugging. To do this, I changed a number of features in my code to be more robust. The first of these features was setting up call features for both gettings my TOF and IMU data as well as my Goal, PID, and initial Speed (and rejecting non-viable values)as shown:
```c++
void GET_IMU_TOF_DATA()
{
  //lab_4 TOF/IMU Code
}
void GET_GPIDS()
{
  bool success;
  success = robot_cmd.get_next_value(POS_goal);
  if (!success){
      Serial.println("ERROR: NOT A REAL distance");
      return;
  }

  success = robot_cmd.get_next_value(Kp);
  if (!success){
      Serial.println("ERROR: NOT A REAL KP");
      return;
  }
  success = robot_cmd.get_next_value(Ki);
  if (!success){
      Serial.println("ERROR: NOT A REAL KI");
      return;
  }
  success = robot_cmd.get_next_value(Kd);
  if (!success){
      Serial.println("ERROR: NOT A REAL KD");
      return;
  }
  success = robot_cmd.get_next_value(PWM);
  if (!success){
      Serial.println("ERROR: NOT A REAL PWM");
      return;
  }
}
```
Previously, I had just copy+pasted from previous labs' cases but felt this was more robust for the feature & more manegeable in the flow. Additionally, I initalized ```serial``` as a global variable so that in my running loop I could add ```central.connected()``` as a condition. This meant if my robot was failing to communicate or I wanted a hard-stop I could turn off BT on my computer and stop it.

Also given all the data being tracked, I cleaned up my code base and commented out the ```temp_tracking[]``` array as it was taking up unnecessary memory.


#### Task X: XXX



## Discussion
In this lab, I learned how to set-up/use the dual motor drivers & run my car. I had some difficulty with disturbance given my sensor placement & wire length, but some fine tuning and rewiring in the future should fix this!

Going forward, I feel more confident about both my soldering & sensor-use in the future as we implement Closed Loop control!

[back](./)

