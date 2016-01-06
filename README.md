# OverclockedSwerve
[![Build Status](https://travis-ci.org/246overclocked/OverclockedSwerve.svg?branch=master)](https://travis-ci.org/246overclocked/OverclockedSwerve)
[![Stories in Ready](https://badge.waffle.io/246overclocked/OverclockedSwerve.svg?label=ready&title=Ready)](http://waffle.io/246overclocked/OverclockedSwerve)

A clean version of our swerve drive code designed to be easily integrated into a new project.

## Installation
All FRC projects use **Apache Ant** to build and deploy code for the robot. In addition, this project also uses Ant to run unit tests. All libraries and dependencies are included in the project, so no external downloads are necessary. The only requirement is to have Ant installed on your computer.

**Determine if you already have Ant installed:** open Terminal (Mac) or Command Line (Windows) and type `ant -version`. If that outputs a version of Ant, then it is already installed and you can skip down to [usage](#usage). If you get something else, you need to install Ant.

### Installing Ant on Mac

On OS X, the easiest way is to [install Homebrew](http://brew.sh), open up a terminal, and type `brew install ant`. (If you have MacPorts installed, you can install Ant with `sudo port install apache-ant`.)

### Installing Ant on Windows

[Download Ant](http://ant.apache.org/bindownload.cgi) and follow the installation instructions, then set up Ant for the command line on Windows (cmd or powershell): 

  1. Add new environment variable `ANT_HOME` and point it to the place where Ant is installed (as Eclipse Plugin or standalone). You can do this temporarily (or for a single session) by using the command `set [variable]=[path]` in cmd or `$env:Variable = 'path'` in powershell, or you can do this permanently by right-clicking on 'My Computer' (or 'This PC') and going to Properties > Advanced system settings (on the left) > Environment Variables. In the window that pops up, you can add the variable `ANT_HOME` equivalent to the Ant installation folder un System variables. 

  1. Append `%ANT_HOME%\bin` to the `Path` envoronment variable under System variables (see the step above) by selecting it in the list of System variables and clicking Edit... - if it is blank, or if you could not find this variable and create it you can simply add `%ANT_HOME%\bin` as the value. If you found it containing a value already, however, **do not delete its contents**, but add `;%ANT_HOME%\bin` to its contents (with a **semi-colon** and **no spaces**). 

## Usage

### Running Ant from the Terminal (Mac) or Command Line (Windows)
Open the terminal (command line on Windows) and `cd` to the root directory of the project (where `build.xml` is located), in this case the `OverclockedSwerve` folder. All Ant commands are of the form `ant [target]` and run in the root directory of the project. Run `ant -p` to see a description of available targets:
```
OverclockedSwerve$ ant -p
Buildfile: [...]/OverclockedSwerve/build.xml
Trying to override old definition of task classloader

Main targets:

 clean         Clean up all build and distribution artifacts.
 compile       Compile the source code.
 debug-deploy  Deploy the jar and start the program running.
 deploy        Deploy the jar and start the program running.
 test          Compile source code and run all junit tests.
Default target: deploy
```
Each "target" is a subcommand you can run. For example `ant deploy` deploys the code to the robot, and is equivallent to deploying from the Eclipse. Most importantly, `ant test` will run automatically run all JUnit tests in the `Overclockedswerve/test` source directory. If no target is specified (by just running `ant` without a target), Ant will default to 'deploy'.

### Running Ant from Eclipse

  1. First make sure you have no Ant Build configurations already set (you can check by going to Run > External Tools > External Tools Configurations... and looking on the left of the window that pops up) and deleting any ones that have already been set. 

  1. Go to File > Import... > select Launch Configurations under Run/Debug > check off the 'OverclockedSwerve' directory on the left (you should see two configuration files checked off on the right) > Finish. 
  **Note:** you may notice that running `git status` after this step reveals that the two configuration files you just imported were deleted - this is completely normal, but be sure to run `git stash` in order to restore these files. 

  1. Right-click on the build.xml file in the OverclockedSwerve directory in the package explorer, select Run As > 1 Ant Build (the first option on the list). 

  1. You should now see two Ant Build configurations on the left of the window that pops up: 'OverclockedSwerve_build.xml' (this one deploys the code to the RoboRIO) and 'OverclockedSwerve_test.xml' (this one just runs all the tests) - select one of them and click 'Run' at the bottom of the window. You will see the output of the Ant Build in the Eclipse output console. 

  1. You can select configurations you've run previously by clicking on the drop-down list next to the deploy button with a red tool chest under it (located just to the right of the usual deploy button) or going back the the 'External Tools Configurations' menu mentioned above. 

## Contributing
Please see [CONTRIBUTING](CONTRIBUTING.md) for more information.

## Credit
  * Thank you to team 4931 for your [awesome Wiki](https://github.com/frc-4931/2014/wiki/Java) explaining FRC Ant usage and installation.
  * Thank you to [Kostya Nazarenko](https://github.com/knazaren) for help in these instructions.
