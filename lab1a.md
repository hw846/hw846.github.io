---
layout: default
---
# Lab 1 Overview:
In this lab, I explore how to both set-up & use a SparkFun RedBoard Artemis Nano (or Artemis henceforth). In this, I go through the device's setup and basic features in 1a and explore the limits of wireless connection in 1b. Written below are my details behind both sections, and more info on the Artmeis can be found [here](https://learn.sparkfun.com/tutorials/hookup-guide-for-the-sparkfun-redboard-artemis-nano/all). All work was done on a 2020 M1 Macbook. 
```Final Wordcount: ???```

## Pre-Lab

For this pre-lab, the only requirement was to install the Arduino IDE. I already had this installed, so there is no figure given but I will show its use for the duration of this write-up.

## Lab 1a Writeup

#### Task 1: Artemis Set-up
I started the lab by first configuring my Artemis in the Arduino environment. In this, I installed the relevant RedBoard Artemis Nano libary (shown here).
![1a_1_Image](figures/1_lab/1a_1.png)

#### Task 2: Blink Program
Following this, to validate the connection and library installation I ran the Blink program (found under ```Files -> Examples -> 01.Blink```)at a baud rate of 115200. The shown video demonstrates this:

<div style="text-align: center;">
  <video width="640" height="480" controls>
    <source src="/figures/1_lab/1a_2.mp4" type="video/mp4">
  </video>
</div>


#### Task 3: Serial Program
To test the serial readout of the board, I uploaded the given "Serial" Sketch to the Artemis (found under ```Files->Examples->Apollo3->Example4_Serial```) where it echoed my inputs as shown into the SerialMonitor:
![1a_3a_Image](figures/1_lab/1a_3a.png)
![1a_4b_Image](figures/1_lab/1a_3b.png)

#### Task 4: AnalogRead Program
To next confirm the temperature readouts by the Artemis, I ran the "analogRead" program (Found in ```Files->Examples->Apollo3->Example2_analogRead```) and was able to get out numerous temperature readings on the SerialMonitor:
<div style="text-align: center;">
  <video width="640" height="480" controls>
    <source src="/figures/1_lab/1a_4.mp4" type="video/mp4">
  </video>
</div>>

#### Task 5: Microphone Output
Finally, to confirm that my Artmeis' microphone worked I uploaded and tested the "MicorphoneOutput" sketch (Found in ```Files->Examples->PDM```). I whistled two different tones and my microphone was able to determine its frequency as shown:

<div style="text-align: center;">
  <video width="640" height="480" controls>
    <source src="/figures/1_lab/1a_5.mp4" type="video/mp4">
  </video>
</div>>

#### Task 6: Three Note Tuner
Given that I am a graduate student, I was additionally tasked with combining the microphone sketch & Serial sketch to create a three note tuner. To do this, I utilized the previous "PDM" sketch and played three different notes "D", "A", and "G" from an online tuner to find the frequency the device read these notes as. From there I plugged these values in as conditionals into the base PDM sketch to get my tuner to work!
```c++
if (ui32LoudestFrequency > 580 && ui32LoudestFrequency < 586)
  {
    Serial.printf("Megalovania 1st Note (D): %d         \n", ui32LoudestFrequency);
  }
  else if (ui32LoudestFrequency > 430 && ui32LoudestFrequency < 438)
  {
    Serial.printf("Megalovania 2nd Note (A): %d         \n", ui32LoudestFrequency);
  }
  else if (ui32LoudestFrequency > 785 && ui32LoudestFrequency < 793)
  {
    Serial.printf("Megalovania 3rd Note (G): %d         \n", ui32LoudestFrequency);
  }
  else
  {
    Serial.printf("Not Megalovania (Tuned Note), tuned note:%d         \n", ui32LoudestFrequency);
  }
```
This code uses a range of values to ensure that slight noise disturbance doesn't affect its ability to read the value. The video below shows this program working!

<div style="text-align: center;">
  <video width="640" height="480" controls>
    <source src="/figures/1_lab/1a_6.mp4" type="video/mp4">
  </video>
</div>>

Also fun fact, the notes identified are the first three notes used in [Undertale's Megalovania](https://www.youtube.com/watch?v=0FCvzsVlXpQ&list=RD0FCvzsVlXpQ&start_radio=1) (hence the comment readouts).
[back](./)
