# RedbackBots 2023 RoboCup Soccer SPL Code Release
This is the official 2023 RedbackBots code release. 
The 2023 code release is published under the `coderelease2023` tag on this repository.
This software is provided 'as is', and the team does not provide public support of this software.

RedbackBots is a undergraduate and postgraduate student team supported by the AI Innovation Lab in the School of Computing Technologies at RMIT University.
The goals of RedbackBots are (1) a research project of the AI Innovation Lab, and (2) to provide education for RMIT students in applications of AI to autonomous robotics systems. 

You may contact the RedbackBots team at: <stem.redbackbots@rmit.edu.au>

Before cloning this repository, you must read our license terms.

## Team Report

Our 2023 team report is included in this repository. The report may be cited as:

    Wiley, T., Thangarajah, J. Bhaskar, V., Avice Demay, J., Ellis, T., Griffiths, S., Hermanto, D., Jain, A.,  Matthews, L., Mullan, L., Owens, M., Toh, K. H., Samarakoon, R., Sun, J., (2023) RedbackBots Team Report - RoboCup Soccer SPL 2023, Technical Report, AI Innovation Lab, School of Computing Technologies, RMIT University, Melbourne, Australia.


## Documentation
Our main documentation is currently internal to our team.
We intend to release more documentation with future code releases.

## Directory Structure
The directory structure is:

* **Config**:
    All configuration files (for building, robots, etc.) are stored here.
    Our scripts link or copy these files to their relevant locations to build and run the software
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

# Acknowledgements

This distribution contains software developed by:
* rUNSWift, UNSW Sydney, Australia (https://runswift.readthedocs.io)
* B-Human, University of Bremen, Germany (http://www.b-human.de)
