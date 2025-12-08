# PID Controlled Ping Pong Ball Levitation

## Introduction
  This experiment of the Ping Pong Levitator aims to demonstrate how a PID (Proportional, Integral, and Derivative) controller works and how each of the variables affects the height of the ping pong ball. Using control systems to balance the input with the actual versus the desired output, the creation and use of the levitator serves as a physical display of how these controllers can be utilized in almost every field of engineering. 

## Control Systems

  <img width="485" height="499" alt="image10" src="https://github.com/user-attachments/assets/8f04db30-47b2-401d-a286-e2f66904d382" />
  
  _Figure 1: Diagrams and example circuits of an open-loop and closed-loop control system. All control systems, regardless of type, have the input, controller, actuator, process, and the output._

  
  Control systems work to regulate the behavior of a device or process through an input and its corresponding desired output. As can be seen in Figure 1, these systems can be open or closed-looped, but both have the basic components: the input, the controller and the signal it has altered, the plant (also known as the actuator and process combined), and the output from the signal. The controller is the mechanism that dictates the output of the system will look like, using certain criteria to achieve the desired output. The plant is composed of two parts often combined into one: the actuator and the process. The actuator is the device in the system that alters or adjusts the environment in response to the signal from the controller. The process is the component of the system that is ultimately controlled or changed in such a way as to produce an output in a desired form. An example of this can be seen in Figure 1. Where the open- and closed-loop systems differ is that closed-loop systems have a sensor element that measures the actual output and compares it to the desired output, allowing the system to then make adjustments to more accurately achieve the desired output; this is the basis of how the levitator will work.

 With the levitator, the system follows the same format: the knob of the device is a potentiometer that serves as a voltage divider, determining the desired height (the input), which is then given to the Arduino (the controller).  The Arduino then takes the voltage, uses user-defined criteria to dictate how the Arduino converts the information into a PWM value, and outputs that value to a motor driver that spins the motor of a fan (the actuator). The force generated from the fan then moves the ball in the acrylic tube with an ultrasound sensor (sensor) on top. The sensor finally sends the height (actual output) of the ball back to the Arduino. It compares the selected height (the desired output) from the potentiometer, which then affects the calculations and output of the Arduino, thereby closing the loop. This closed-loop system that the levitator utilizes not only 

## PID Controller

<img width="461" height="397" alt="image6" src="https://github.com/user-attachments/assets/0156d513-a595-4a3f-8a01-977dfa4559e2" />

_Figure 2: The Flow/Block Diagram of the ping pong ball levitator. The interaction between all of the components, along with what information they transmit to each other, is here as well._


  A PID controller computes a control signal based on three components: proportional, integral, and derivative control. Each of these components work to change the characteristics of the output, as shown in Figure ?. The proportional term produces an output value that is proportional to the current error value. This causes an overall decrease in the rise time and steady-state error, but increases the initial overshoot of the output. The integral term works by staying proportional to both the magnitude and duration of the error. This helps to zero out the steady-state error and lead the output to be a smaller range of values, but also increases the overshoot value. The derivative term responds to the rate of change of the error and multiplies it by an accompanying coefficient, allowing the system to predict its behavior accordingly.  This can decrease the overshoot and stabilize the system, but can potentially amplify measurement in the noise. The components of this controller can be expressed in both the time domain [Eq. 1] and Laplace domain [Eq. 2]. As stated above, the Arduino will serve as the controller, working to balance all of the coefficients as it takes in the desired height and works to move the ball with the characteristics that it is given.

## Hardware
### Components
- 1 x 25k Ω Potentiometer
- 1 x 220 Ω Resistor
- 1 x 12 V DC Fan
- 1 x Acrylic Tube
- 1 x HCSR04 Ultrasonic Sensor
- 1 x Arduino Uno
- 1 x Ping Pong Ball
- 1 x Tiny Nut
- Tape
- 1 x DRV 8871 Motor Driver
- 1 x Nail
- 1 x Kastar 12V 6A Power Supply

Using a prebuilt wooden stand, the base of the acrylic tube is attached to the fan. A nail is put through the tube as the platform to hold the ping pong ball above the fan without touching it. The ping pong ball, with the small nut attached using tape, is dropped inside. The tube is then covered with the ultrasonic sensor on the top and held in place with tape. At the base of the stand, the potentiometer and the resistor are connected as a voltage divider between 5V and GND. The output voltage from the voltage divider is then connected to the analog input pin 5 in the Arduino. The motor driver’s In1,  In2, and GND pins are connected to the digital pins 10 and 11 and the GND pin of the Arduino, with the driver power being connected to the power supply and the motor power being attached to the power and ground of the fan. The ultrasonic sensor’s power is connected to the 5V and GND of the Arduino. The echo pin is connected to the digital input pin 3, and the trigger pin is connected to the digital input pin 2 on the Arduino.


## Goal

The goal of this project is to use PID control to regulate the target height and maintain a ping-pong ball at a target height within a vertical wind tunnel. By managing the coefficients of different aspects of the controller, the ball should be able to reach its intended height quickly and without too much error

- Methodology
- Describe the Hardware + photos
- Describe the software
  - Sensor (ultrassound)
  - Actuator ()
  - PID controller (library)

- Show tests to adjust PID controller using manual tunning
- Comment the best results

- gif to show the curves and the ping pong ball

## Conclusion


## Appendix
# Equations
1. $ controller(t) = u(t) = K_p e(t) + K_i\int e(t) \mathrm{d}t + K_d$
2. $




