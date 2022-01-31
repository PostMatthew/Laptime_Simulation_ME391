# Laptime_Simulation_ME391
 This is a repo for laptime simulation project.

## Project Description:

This project is to develop a lap time simulation tool to be used for the Lafayette Motorsports project. The lap time simulation will allow for Lafayette Motorsports to input GPS data for a given track and relevant car parameters and have returned their projected lap time around the track for varying driver styles. The project is split into three tentative phases (w/an additional phase 0 for initial administrative work):

### Phase 0: Administrative Setup
Description: This phase serves as a primer for the first two week of the semester for any necessary administrative work. This phase should be completed as soon as possible in order to commence with the project. 
Validation Method: Not Applicable 
Tentative Completion Date(s): 2/1

### Phase 1: Finish Point Mass Racer
Description: In ME 497: Senior Design I, Matt began work on a point mass simulation based on a homework assignment for Professor Brown’s ME 477: The need for speed course. This initial phase will encompass finishing the point mass simulation (PMS) and incorporating additional scripts written by Professor Brown (for steady speed cornering) and Harry Ma ME ‘22 (for engine dynamics). 
Validation Method: Validation will occur in CarSim in order to compare accuracy of simulation.
Tentative Completion Date(s): Construction: 2/21, Validation: TBD

### Phase 2: Develop Driver Models
Description: This phase will serve to develop driver models based on a literature review. These models will add additional complexity and more accurately predict the lap time of the car. Driver models will be developed based on a literature review of published vehicle dynamic and automotive journals. This phase serves as the end of the simulation/computational work on this project. At the completion of this phase Lafayette Motorsports will be able to run an Octave Jupyter notebook or MATLAB script to be able to determine the lap time of the car. 
Validation Method: Validation will occur in CarSim to compare simulation accuracy and driver model accuracy. 
Tentative Completion Date(s): Construction: 3/25, Validation: 4/1

### Phase 3: Convert to Web Application
Description: This phase will serve to incorporate the simulation in a java script web app. The web application will allow for a user to upload GPS data for a track and car parameters and have returned the car’s lap time, simulated acceleration, velocity, and other relevant parameters. Simulated data will be exportable as a text file for future access. Visually, the user of this application will be able to see an animation of their car going around their track and real time plots of selectable parameters of their car. An additional goal (but not full requirement) for this application will also be to allow for the user to establish multiple configurations of the car and see differences in car performance. (Ex: how does changing the roll stiffness impact lap time) Full metrics/constraints of the application are discussed in the metric and constraints document.
Validation Method: Case testing the application for varying configurations of car parameters and tracks.
Tentative Completion Date(s): Construction: 5/1, Validation: 5/8
