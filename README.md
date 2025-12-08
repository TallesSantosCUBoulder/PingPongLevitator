# PID Controlled Ping Pong Ball Levitation

## Introduction
  This experiment of the Ping Pong Levitator aims to demonstrate how a PID (Proportional, Integral, and Derivative) controller works and how each of the variables affects the height of the ping pong ball. Using control systems to balance the input with the actual versus the desired output, the creation and use of the levitator serves as a physical display of how these controllers can be utilized in almost every field of engineering. 

## Control Systems

  <img width="485" height="499" alt="image10" src="https://github.com/user-attachments/assets/8f04db30-47b2-401d-a286-e2f66904d382" />
  *Figure 1: Diagrams and example circuits of an open-loop and closed-loop control system. All control systems, regardless of type, have the input, controller, actuator, process, and the output.*
  
  Control systems work to regulate the behavior of a device or process through an input and its corresponding desired output. As can be seen in Figure 1, these systems can be open or closed-looped, but both have the basic components: the input, the controller and the signal it has altered, the plant (also known as the actuator and process combined), and the output from the signal. The controller is the mechanism that dictates the output of the system will look like, using certain criteria to achieve the desired output. The plant is composed of two parts often combined into one: the actuator and the process. The actuator is the device in the system that alters or adjusts the environment in response to the signal from the controller. The process is the component of the system that is ultimately controlled or changed in such a way as to produce an output in a desired form. An example of this can be seen in Figure 1. Where the open- and closed-loop systems differ is that closed-loop systems have a sensor element that measures the actual output and compares it to the desired output, allowing the system to then make adjustments to more accurately achieve the desired output; this is the basis of how the levitator will work.

 With the levitator, the system follows the same format: the knob of the device is a potentiometer that serves as a voltage divider, determining the desired height (the input), which is then given to the Arduino (the controller).  The Arduino then takes the voltage, uses user-defined criteria to dictate how the Arduino converts the information into a PWM value, and outputs that value to a motor driver that spins the motor of a fan (the actuator). The force generated from the fan then moves the ball in the acrylic tube with an ultrasound sensor (sensor) on top. The sensor finally sends the height (actual output) of the ball back to the Arduino. It compares the selected height (the desired output) from the potentiometer, which then affects the calculations and output of the Arduino, thereby closing the loop. This closed-loop system that the levitator utilizes not only 

- PID controller
- Hardware
  - Input (sensor)
  - Output (actuator)
  - Controller

## Goal
- Build the PingPOngLevitator

- Methodology
- Describe the Hardware + photos
- Describe the software
  - Sensor (ultrassound)
  - Actuator ()
  - PID controller (library)

- Show tests to adjust PID controller using manual tunning
- Comment the best results

- gif to show the curves and the ping pong ball

Conclusion




