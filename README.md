# Concurrent Goal Assignment and Collision Free Avoidance (CGA_CFA) Package
The objective of the package is to find goal assignments for 2D formation of aerial vehicles that guarantee collision free trajectories while minimizing the flight time.
The package uses the "Concurrent Goal Assignment and Collision-Free Trajectory Generation for Multiple Aerial Robots" paper by Benjamin Gravell and Tyler Summers to develop python scripts that assign N aerial robots to N desired locations based on a one-to-one mapping that ensures minimal time-in-flight.

## Package Requirements
The package is composed on Python scripts and modules. Aside from the standard Python libraries, the package requires Scipy. To get scipy, follow the instruction in [https://www.scipy.org/install.html](https://www.scipy.org/install.html).

The package was tested in a Linux environment. The sample usage in `cga_cfa_alt.py` might not work because it uses `/` for paths, but the class itself should still be supported.

## Package Installation
Either download the zip file and extract at a desired location or clone the package using `git clone https://github.com/The-SS/cga_cfa.git` in a terminal window. Nothing else is required.

## Using the Package
A sample usage is provided through `main` in `cga_cfa_alt.py`. You can execute this script and it should print out the output of solving the assignment problem.

You can also import the `CGA_CFA_ALT` class in a Python script and use the provided class API to update the parameters, solve the assignment problem, and retrieve the relevant results. The API and its usage are explained in `cga_cfa_alt.py`.
