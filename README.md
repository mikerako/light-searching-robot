# Light Searching Robot with Obstacle Avoidance

Michael Rakowiecki

## Overview

Small robot built on Pololu chassis using NUCLEO F401RE to communicate with sensors and implement state machine

## Sensors

The sensors used for this project are:

1. PC8591 module (light sensor)		x1
2. SG90 Servo						x1
3. SRF04 Ultrasonic sensor			x1
4. DRV8835 H-Bridge					x2
5. Zumo Shield for Arduino			x1
6. NUCLEO-F401RE					x1

## Purpose

This robot was made to demonstrate competence in course material involving serial bus communication, MMIO, ADC/DAC conversion, and general embedded system common knowledge. 

The robot has a state that searches for the brightest light source around it and drives to it while avoiding obstacles in its way.

## Usage

Once flashed, the only required operation is turning the switch on the Zumo Shield from off to on