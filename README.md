ATC Radar Simulator (Ada)

Author: Tisha Patel

Date: June 2025

Contact: \[tishaapatel08@gmail.com]



###### **Overview**



This project is a real-time Air Traffic Control (ATC) radar simulator written in Ada. It simulates the tracking and management of multiple aircraft, enforcing aviation safety standards such as ICAO separation and wake turbulence rules. The simulator integrates my experience as a private pilot and my technical background in computer engineering, demonstrating a strong interest in aerospace, simulation, and safety-critical systems.





###### **Features**



> **Real-Time Radar Simulation:**

&nbsp; Tracks 3 aircraft with 1-minute radar sweeps over a 10-minute simulation.



> **Flight Plan Navigation:**

  Each aircraft follows a predefined flight plan with waypoints, speed, and altitude restrictions.



> **ICAO \& RVSM Separation Enforcement:**

&nbsp; Enforces minimum horizontal and vertical separation, including RVSM (Reduced Vertical Separation Minimum) above FL290.



> **Wake Turbulence Management:**

  Implements wake turbulence separation for Heavy, Medium, and Light aircraft categories.



> **Flight Phase Monitoring:**

  Tracks aircraft through all flight phases: Taxi, Takeoff, Climb, Cruise, Descent, Approach, Landing.



> **Conflict Detection \& Advisory Generation:**

&nbsp; Detects separation conflicts and issues resolution advisories in real time.



> **ETA Calculation:**

  Displays estimated time of arrival to the next waypoint for each aircraft.



> **Waypoint Progress Tracking:**

  Shows all waypoints and highlights current progress for each aircraft.



> **Event Logging:**

  Logs speed and altitude changes at each waypoint.



> **Radar Display:**

  Visual ASCII radar screen with aircraft symbols, range rings, and real-time updates.





**Technologies \& Concepts**

---

> **Language:** Ada



> **Domain:** Air Traffic Control, Aviation Safety, Real-Time Simulation



> **Key Concepts:** State management, type safety, procedural programming, aviation standards





###### **Aircraft Simulated**



 AAL123: Heavy, cruising at FL300, heading East



 UAL456: Medium, descending FL305 → FL260, heading West



 DAL789: Heavy, climbing FL300 → FL420, heading South





###### **How to Run**



**Requirements:** Ada compiler (e.g., GNAT).



**Clone the repository:**



git clone https://github.com/tishap27/atc\_simulator.git

cd atc\_simulator


**Build the project:**



gnatmake atc\_simulator.adb



**Run the simulator:**



./atc\_simulator




**Project Motivation**

---

As a student in Computer Engineering Technology and a licensed private pilot, I created this project to combine my passion for aviation with my interest in safety-critical software. The simulator demonstrates both technical and domain-specific skills relevant to aviation/aerospace and ATC systems.





###### **Contact**

For feedback, collaboration, or volunteering opportunities, please contact:

Tisha Patel

**\[tishaapatel08@gmail.com]**

