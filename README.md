# Hydraulically Actuated Industrial Systems
This project provides a machine learning-based library for modeling non-ideal joints in mechanical systems. 
It aims to replace traditional joint models using pre-trained ML models.

## Features
- Types – Support for different non-ideal joint types (Generic joint)
- Properties – Flexibility, friction, contact, wear, tolerances, backlash, compliance, lubrication, fretting, misalignment, vibration    
- Exudyn – Data acquision and preprocessing tools (https://exudyn.readthedocs.io/en/v1.9.83.dev1/)

## Dependencies 
- numpy stl (https://pypi.org/project/numpy-stl/)
- Abaqus (https://www.3ds.com/products/simulia/abaqus)

## Folders and files
- Main file – 'MainScript.py' 
- Exudyn – Models/..
- ML –  ML/..
- Results – Solutions/.., ../data/, ../results/, ../MLmodels/
- Latex – Figures/.., References/.., 'LayOut.cls','Manuscript.tex'  

## Contact

- Qasim Khadim – University of Oulu (qasim.khadim@lut.fi)
- Johannes Gerstmayr – University of Innsbruck (johannes.gerstmayr@uibk.ac.at)

## License
- These models are free to use. 

## References
- Use the references below for citing flexible bodies, flexible multibody systems, hydraulics, actuator frictions, and hydraulically flexible multibody systems. In particular, Reference v combines all these models.
	1. Andreas Zwölfer and Johannes Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction: How to calculate the invariants without a lumped mass approximation. Acta Mechanica, 232:835–851, 2021.
	2. Johannes Gerstmayr, Stefan Holzinger, and Andreas Zwölfer. From 3D solid finite elements to reduced flexible multibody bodies with constraint interfaces: A holistic approach. Journal of Sound and Vibration, Under Review, 2025.
	3. Johannes Gerstmayr. Exudyn– A C++ based Python package for flexible multibody systems. Multibody System Dynamics, 60(4):533–561, 2023.
	4. Rolf Isermann. Mechatronic systems—innovative products with embedded control. Control Engineering Practice, 16(1):14–29, 2008.
	5. Brian Armstrong-Helouvry. Control of machines with friction, volume 128. Springer Science & Business Media, 1991.
	5. Qasim Khadim, Peter Manzl, Emil Kurvinen, Aki Mikkola, Grzegorz Orzechowski, Johannes Gerstmayr. Real-Time Structural Deflection Estimation in Hydraulically Actuated Systems Using 3D Flexible Multibody Simulation and DNNs. Mechanical Systems and Signal Processing, Under Review, 2025.

## Installation
Clone the repository and install dependencies:
```bash
git clone https://github.com/qkhadim22/Exudyn-hydraulically-actuated-industrial-systems.git