# IoT Final Project - Detection and Containment of COVID-19 in Assisted Living Spaces
----------------------------------------------------------------------------------------------------------------------------------------------------

## Authors - Pavan Shiralagi, Gitanjali Suresh and Sarayu Managoli

## Professor - Tim Scherr

## Contents

This repository contains the Friend Node implementation for our final project.

Demo video - [Project Demo](https://drive.google.com/drive/u/1/folders/1ZPNZiMcrfCPBDFvCzMxLzUf7_b3DdGpf "Project Demo")

Repository with Caretaker LPN code - [Caretaker LPN](https://github.com/CU-ECEN-5823/final-project-assignment-Gitanjali-Suresh "Caretaker LPN")

Repository with Patient LPN code - [Patient LPN](https://github.com/CU-ECEN-5823/final-project-assignment-sarayumanagoli "Patient LPN")

Our teams Google Drive folder can be found here - [COVID19Detection](https://drive.google.com/drive/folders/1ZPNZiMcrfCPBDFvCzMxLzUf7_b3DdGpf?usp=sharing "COVID19Detection")

Pavan Shiralagi's individual report can be found here - [Pavan Shiralagi](https://drive.google.com/drive/u/1/folders/1rVscWxk9fUWKUg6-Ww8QdJyeg4_S-5me "PavanShiralagi")

The goal of this project is to create a prototype of a Bluetooth Mesh system to aid in the detection and containment of the severe acute respiratory syndrome coronavirus 2 (SARS-CoV-2) 
causing the deadly coronavirus disease (COVID-19).

-----------------------------------------------------------------------------------------------------------------------------------------------------
## Project Update 1
Working as Subscriber or Publisher, Factory reset working, PB0 interrupt enabled and published, display integrated.

-----------------------------------------------------------------------------------------------------------------------------------------------------
## Project Update 2
Friend Node implemented, can work as either Publisher or Subscriber. Interface with PIR sensor completed and works with Friend Node as Subscriber.
Integration of Humidity Sensor in I2C state machine in Friend Node as Subscriber works. Friend Node as Subscriber or Publisher tested with LPN sending/receiving
button press.

-----------------------------------------------------------------------------------------------------------------------------------------------------
## Final Update
The Friend Node is fully functional, as described in the [Final Project Report](https://drive.google.com/drive/u/1/folders/1ZPNZiMcrfCPBDFvCzMxLzUf7_b3DdGpf "Final Project Report")
This node has the following functionality.
- Friend Node
	- Implemented PIR sensor to detect unauthorized entry
	- Implemented humidity sensor to control humidistat
	- Reduces power usage by load power management while using humidity sensor and sleep modes implemented. Disables interrupts from PIR if the caretaker enters the room
	- Receives and processes data from both LPNs using Generic OnOff Server and Generic Level Server to raise the following alerts (Clears alerts using pushbutton (PB0)),
		- Patient fainted (accelerometer reading - LPN 1)
		- High Temperature (temperature sensor - LPN 1)
		- Unauthorized person (button value from LPN 2 and PIR value)
	-Uses persistent data to store,
		- Highest temperature received
		- Whether caretaker is inside or outside the room
		- Number of alerts received


-----------------------------------------------------------------------------------------------------------------------------------------------------