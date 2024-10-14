# RedbackBots 2024 RoboCup Soccer SPL Code Release
This is the official 2024 RedbackBots code release. 
The 2024 code release is published under the `coderelease2024` tag on this repository.
This software is provided 'as is', and the team does not provide public support of this software.

RedbackBots is a undergraduate and postgraduate student team supported by the AI Innovation Lab in the School of Computing Technologies at RMIT University.
The goals of RedbackBots are:

1. A research project of the AI Innovation Lab, and
2. To provide education for RMIT students in applications of AI to autonomous robotics systems. 

You may contact the RedbackBots team at: <stem.redbackbots@rmit.edu.au>

Before cloning this repository, you must read our license terms.

## Team Report

Our [2024 team report](RedbackBots_Teampaper_2024.pdf) is included in this repository. The report may be cited as:

    Wiley, T., Thangarajah, J., Abothu, R., Avice Demay, J., Bajaj, H., Chan, H. Y. S., Cao, M., Ellis, T., Field, M., Ganji, R., Griffiths, S., Lamb, C., Lohani, P., Mullan, L., Owens, M., Pauckner, J., Perez, R., Putter, M., Sandoval Rodriguez, A., Stephens, J., and Thom, J.
    (2024) RedbackBots Team Report - RoboCup Soccer SPL 2024, Technical Report, AI Innovation Lab, School of Computing Technologies, RMIT University, Melbourne, Australia.

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

# License

Before cloning this repository, you must read our [license terms](License.md).

# Acknowledgements

This distribution contains software developed by:
* rUNSWift, UNSW Sydney, Australia (https://runswift.readthedocs.io)
* B-Human, University of Bremen, Germany (http://www.b-human.de)

----

Copyright (c) 2024.

Artificial Intelligence Innovation Lab, Centre for Industrial AI and Research Innovation (CIAIRI), School of Computing Technologies, STEM College, RMIT University.
