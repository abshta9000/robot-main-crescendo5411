# ![Prosper-Engineering-Team](/Banner.png)
# Robot-Main-Crescendo: Robotalons Crescendo Season Code

[![Build Status](https://github.com/FRC5411/robot-main-crescendo5411/actions/workflows/Build-Push.yml/badge.svg?branch=Production)](https://github.com/FRC5411/robot-main-crescendo5411/actions/workflows/Build-Push.yml)
[![GitHub Contributors](https://img.shields.io/github/contributors/FRC5411/robot-main-crescendo5411.svg?branch=Production)](https://github.com/FRC5411/robot-main-crescendo5411/graphs/contributors)
[![GitHub Issues](https://img.shields.io/github/issues/FRC5411/robot-main-crescendo5411.svg?branch=Production)](https://github.com/FRC5411/robot-main-crescendo5411/graphs/issues)
## Installation

Below is a list of instructions to properly build the project, see [requirements](##Requirements)

1. Clone the repository with `git clone https://github.com/FRC5411/robot-main-crescendo5411.git`
2. Build the repository with `./gradlew build` or `./gradlew build` if you do not have a local gradle installation

## Requirements

- [JDK 8](https://adoptium.net/temurin/releases/?version=8)
    - Ubuntu: run `sudo apt install openjdk-8-jdk`
    - Windows: install the JDK 11 .msi from the link above
    - macOS: install the JDK 11 .pkg from the link above
- [Gradle 7 (Optional)](https://gradle.org/releases/)
    - Follow Gradle's installation [guide](https://gradle.org/install/#prerequisites)
- [WPI VSCode (Optional)](https://github.com/wpilibsuite/allwpilib/releases/tag/v2023.4.3)
    - Follow WPILib's [guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- [AdvantageScope (Optional)](https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v3.0.0)
    - Execute the distribution to begin setup
      
## Structure & Organization

The project is organized based on WPILib's command-based control structure modified to fit our own needs:

```
└──Root-Directory
  ├──.gradle: Gradle build files, should be in .gitignore
  ├──.run:  Project configurations provided to deploy and simulate the project regardless of IDE being used. (Optional)
  ├──.vscode: Personal workspace configurations and launch configurations if you chose not to use .run configurations; in which case this should not be in .gitignore
  ├──.wpilib: WPILib project settings, preferences like language, year, and team number here.
  ├──bin: Gradle build binaries
  ├──build: Compiled build files.
  ├──gradle: Gradle wrapper executable and properties for executing builds without a Gradle installation on a system, use this instead of Gradle when │available.
  ├──src/main: Source directory where all working code should be.
  |        ├──deploy: Where all files that need to be deployed to the RoboRIO, like pathplanner files, text files, etc need to be kept for usage in deployed code.    
  │ 	     └──java
  |           ├──main/org/robotalons
  |           |    	 ├──crescendo: Where working robot code should go, everything here is stuff that actually needs to be compiled and deployed to the robot.
  │ 	        |       |     ├──subsystems: Contains a folder with each subsystem, where a static subsystem instance class is paired with constants that it uses directly.
  │ 	        |       |     |   ├──ExampleSubSystemDir/ExampleSubsystem.java: Should Actually implement some sort of accessor for obtaining an internal, private 	     
  │ 	        |       |     |   |  reference to a static instance of the subsystem, used for commands, and should inherit from an Base class.
  │ 	        |       |     |   ├──ExampleSubSystemDir/ExampleSubsystemSimulation.java: Should implement for sim usage only some sort of accessor for obtaining an internal, private 	     
  │ 	        |       |     |   |  reference to a static instance of the subsystem, used for commands, and should inherit from an Base class.
  │ 	        |       |     |   ├──ExampleSubSystemDir/ExampleSubsystemBase.java: An abstract class containing the nesscessary methods or both sim and real robot modes that inheirits
  │ 	        |       |     |   |  from Subsystembase
  │ 	        |       |     |   └──ExampleSubSystemDir/Constants.java: Example, should contain constants relevant only to the ExampleSubsysten.
  │ 	        |       |     ├───RobotConstants.java: Global constants for the project.
  │ 	        |       |     ├───Main.java: Where runtime originates from, all java projects start here,
  │ 	        |       |     ├───Robot.java: Al robot-wide methods for periodic, init, and exit, along with where a static Container instance should be grabbed from to start 
  │ 	        |       |     |   the robot operations
  │ 	        |       |     └──RobotContainer.java: Container for Controller and Piloting system configurations at launch, should ideally get a reference to each static         
  │ 	        |       |         instance of all subsystems to initialize them.
  |           |       └──lib: Where files used in the project that are abstracted enough to be used on other projects should go, reusable code.
  │ 	        └──test: Contains test files using a framework like JUnit to test different parts of the code to see if they work as intended.
  ├──vendordeps: All dependencies utilized for the project, such as CTRE and REVLib
  ├──.gitignore: Git Ignores all files and directories within it’s contents.
  ├──build.gradle: Defines the build specifications for Gradle when this project is built, this shouldn’t be modified under most circumstances unless it’s │dependencies.
  ├──settings.gradle: Contains project-wide Gradle settings
  └──WPILib-Liscense.md: Legal stuff for project usage
```
