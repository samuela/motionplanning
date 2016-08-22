motionplanning
==============

This is a repository containing research code attempting to connect pyDPMP and the Open Motion Planning Library (OMPL). More generally, this repository is home-base for an effort to develop new motion planning algorithms built on D-PMP.

The code currently reflects my (@samuela aka Sam Ainsworth) personal setup, and may require some tinkering on other environments. Feel free to reach out if you have any difficulties or questions!

Manifest
--------

### Jupyter Notebooks
* `basic_circle.ipynb` An illustration of D-PMP solving an elementary motion planning problem. No OMPL involved.
* `grid.ipynb` A grid environment.

### Python code
* `DPMPPlanner.py` Contains an implementation of an OMPL-compatible solver
written using OMPL's Python bindings. Path solving done via D-PMP.
* `ompl_app_dpmp.py` Is a Frankensteined version of OMPL's `ompl_app.py` GUI application. The original GUI code is very messy and OMPL is not flexible enough to allow planners to be written in Python without altering the GUI code. It may be worth diffing this file against the canonical version in order to understand what changes have been made.
* `ompl_app.py` A mostly consistent version of the original OMPL version, with a number of fixes and code cleanliness adjustments.
* `path_check_bug.py` Illustrates a SEGFAULT error in the OMPL Python bindings. Bug should be patched now. Not important.

### Dockerfiles and scripts
I (Sam Ainsworth) run OS X. This makes compiling, running, and developing OMPL is a real pain in the ass if not downright impossible. In order to make things more approachable, I've developed a number of Docker images in order to run OMPL. These are accompanied by bash scripts that facilitate running the Docker containers and setting up the GUI to run on OS X.

#### Dockerfiles
* `docker/ompl-buildenv/` establishes all of the necessary packages and system configuration in order to compile OMPL.
* `docker/ompl-official` is based on the ompl-buildenv image and builds the official OMPL 1.1.1 from scratch.
* `docker/ompl-devenv` is based on the ompl-official image and adds some nice packages for development, like IPython, matplotlib, and numpy.

#### Scripts
Running Docker containers with X11 on OS X is a real pain. But unfortunately it's necessary in order to run the GUI. The two `run_ompl-devenv.sh` and `run_ompl-official.sh` scripts handle all of the setup and teardown necessary to run the Docker images with X11 forwarding. They also handle attaching folders on the host machine to the file system of the container and the installation of the pyDPMP pip package in the container. They are customized to run on my particular machine, but should be fairly easy to adapt to other configurations.
