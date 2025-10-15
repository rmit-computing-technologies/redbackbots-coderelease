# RedbackBots 2025 RoboCup Soccer SPL Code Release
This is the official 2025 RedbackBots code release.
The 2025 code release is published under the `coderelease2025` tag on this repository.
This software is provided 'as is', and the team does not provide public support of this software.

RedbackBots is a undergraduate and postgraduate student team supported by the AI Innovation Lab in the School of Computing Technologies at RMIT University.
The goals of RedbackBots are:

1. A research project of the AI Innovation Lab, and
2. To provide education for RMIT students in applications of AI to autonomous robotics systems.
3. Outreach in STEM (Science, Technology, Engineering and Mathematics), promotion of the benefits of Autonomous Robotics within the public view, and encouraging primary and secondary school students to pursue studies in STEM fields.

You may contact the RedbackBots team at: <stem.redbackbots@rmit.edu.au>

Before cloning this repository, you must read our license terms.

## Team Report

Our [2025 team report](TeamReports/RedbackBots_Teampaper_2025.pdf) is included in this repository. The report may be cited as:

    Wiley, T., Thangarajah, J., Babayigit, S., Chatham, M., Delgado, K., Ellis, T., Field, M., George, B., Griffiths, S., Hanidar, S., Hong, P., Hookmani, Y., Killeen, B., Kulathunga, T., Laurentia, K., Lohani, P., Mullan, L., Owens, M., Pham, M. H., Phillips, C., Sandoval, A., Theofilas, J, Thomson, J., Verma, R., Zhen, K. Z.
    (2025) RedbackBots Team Report - RoboCup Soccer SPL 2025, Technical Report, AI Innovation Lab, School of Computing Technologies, RMIT University, Melbourne, Australia.

Our past team reports can be found in the [TeamReports](TeamReports).

## Documentation
Limited documentation is provided in [Docs](Docs/Home.md).

## Directory Structure
The directory structure is:

* **Config**:
    All configuration files (for building, robots, etc.) are stored here.
    Our scripts link or copy these files to their relevant locations to build and run the software
* **Docs**:
    Contains some pages from our internal team wiki. It is stripped down to include only steps on getting our code running on a robot.
* **Install**:
    Software and scripts for installing the RedbackBots software on a Nao robot. Static files that are deployed to the Nao. These include operating system configurations, static robot configuration files for use in the RedbackBots software, audio files, and machine learnt models. The Nao image and contents of this directory are synced to the robot with nao_sync.
* **Make**:
    Software and scripts for building the RedbackBots software, primarily consisting of CMake configuration files. Also includes scripts for generating the bootable USB image.
* **Make/Common**:
    This is where any executables are stored, such as configuration scripts (buildSetup, nao_sync)
* **Src**:
    C++ and Python source files of the RedbackBots codebase.
* **Src/behaviours**:
    This is the source code for behaviours, including the python infrastructure.
* **Src/robot**:
    This is the C++ source code for the redbackbots binaries.
* **Util**:
    Utility files, at present only containing pre-compiled libraries for various operating systems architectures, including x86_64 and arm64.


# Writing Your First Behaviour

We have included a 'broken' Demo.py bahaviour as an example for where you can
start, to get started try updating this file with the objective of having the robot move towards the ball and kick it. The base configurations are in place for you to start working with.
The file to be corrected is demo.py, where we deliberately kept some issues for you.


* Modify:
Modify demo.py in order to get the robot walk towards the ball and kick it

* Execute:
The first step is to sync the change in code to the robot by executing the below command.


```shell
$ nao_sync -r <robot_name>
```
Once synced, ssh to the Robot which will take you to the robot terminal, and run the below command to run the code.


```shell
$ redbackbots -s demo
```

# License

Before cloning this repository, you must read our [license terms](License.md).

# Acknowledgements

This distribution contains software developed by:
* rUNSWift, UNSW Sydney, Australia (https://runswift.readthedocs.io)
* B-Human, University of Bremen, Germany (http://www.b-human.de)
* Nao Devils, TU Dortmund University, Germany (https://naodevils.de)

----

Copyright (c) 2025.

Artificial Intelligence Innovation Lab, RMIT Artificial Intelligence Advanced Innovation Experience Hub (RAIsE Hub), School of Computing Technologies, STEM College, RMIT University.
