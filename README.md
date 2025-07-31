# Hydraulically Actuated Industrial Systems
This project provides a machine learning-based library for modeling non-ideal joints in mechanical systems. 
It aims to replace traditional joint models using pre-trained ML models.

## Features
- Types – Hydraulically actuated rigid and flexible systems (Lift Boom and Crane)
- Exudyn – Data acquision and preprocessing tools (https://exudyn.readthedocs.io/en/v1.9.83.dev1/)

## Dependencies 
- numpy stl (https://pypi.org/project/numpy-stl/)
- Abaqus (https://www.3ds.com/products/simulia/abaqus)

## Folders and files
- Graphics and mesh – AbaqusMesh/... 
- Exudyn – Models/.., 'Container.py' (Parameters), 'Control.py' (Control Signals)  
- Results – solutions/.., ../OneArm/ (LiftBoom), ../TwoArms/ (PATU crane)

## Contact

- Qasim Khadim – University of Oulu (qasim.khadim@lut.fi)
- Johannes Gerstmayr – University of Innsbruck (johannes.gerstmayr@uibk.ac.at)

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