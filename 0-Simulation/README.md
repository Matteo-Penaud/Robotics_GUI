<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" property="dct:title">Robotics initiation class materials</span> by <span xmlns:cc="http://creativecommons.org/ns#" property="cc:attributionName">Passault Grégoire, Olivier Ly and Remi Fabre</span> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.

# Simulation
A simple simulation based on PyBullet.

# Requirements
Tested on Python 3.6 but should work on any recent version of Python:
```bash
pip install numpy pygame pybullet onshape-to-robot transforms3d scipy termcolor
```

# Usage
To run manual tests, use :
```bash
python3 kinematics.py
```
To run the simulation in direct mode, use :
```bash
python sim2.py --mode direct
```
To change mode, replace direct by the following arguments :
- inverse, to run inverse kinematics
- circle, to run the circle mode (your robotic arm will draw a circle) - (WORK IN PROGRESS)
- triangle, to run the triangle mode (your robotic arm will draw a triangle) - (WORK IN PROGRESS)
