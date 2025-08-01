# Hydraulically Actuated Industrial Systems
This project provides hydraulically actuated rigid and flexible crane models. The actual hydraulic parameters and geometric configuration align with existing PATU crane, available at the Laboratory of Intelligent Machines, LUT University. 
The dynamic responses of these models have been validated against experimental data.

## Details
- Rigid Multibody System Dynamics — Redundant and Minimal Coordinate Methods
- Flexible Multibody System Dynamics — Component Mode Synthesis 
- Hydraulics – Lumped Fluid Theory
- Actuator friction – Velocity-dependent Stribeck friction model

## Dependencies 
- Exudyn (https://exudyn.readthedocs.io/en/v1.9.83.dev1/)
- numpy stl (https://pypi.org/project/numpy-stl/)
- Abaqus (https://www.3ds.com/products/simulia/abaqus)

## Folders and files
- Parameters – 'Models/Container.py'
- Controls – 'Models/Control.py' 

## Acknowledgements
The author would like to thanks Prof. Johannes Gerstmayr for hosting me at University of Innsbruck, Department of Mechatronics, Innsbruck, Austria. His supervision enabled me to understand and write this code in Exudyn environment. Special thanks to Stefan Holzinger and Michael Pieber.
I would also like to thanks Prof. Grzegorz Orzechowski, Prof. Emil Kurvinen, Prof. Aki Mikkola and Prof. Heikki Handroos for supporting this work.

## License
- These models are free to use. 

## References
- Use the references below for citing the hydraulic parameters and hydraulically actuated flexible multibody systems. Contributions on flexible bodies, flexible multibody systems, hydraulics, and actuator frictions can be found in reference ii.
	1. Qasim Khadim, et al. Experimental investigation into the state estimation of a forestry crane using the unscented Kalman filter and a multiphysics model. Mechanism and Machine Theory, 189, 105405, 2023.
	2. Qasim Khadim, Peter Manzl, Emil Kurvinen, Aki Mikkola, Grzegorz Orzechowski, Johannes Gerstmayr. Real-Time Structural Deflection Estimation in Hydraulically Actuated Systems Using 3D Flexible Multibody Simulation and DNNs. Mechanical Systems and Signal Processing, Under Review, 2025.

## Installation
Clone the repository and install dependencies:
```bash
git clone https://github.com/qkhadim22/Exudyn-hydraulically-actuated-industrial-systems.git