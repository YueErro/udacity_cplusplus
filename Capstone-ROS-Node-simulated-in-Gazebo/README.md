# Capstone - ROS Node simulated in Gazebo
The README describes the project you have built.

## Table of contents
* [Project file and class structure](#Project-file-and-class-structure)
* [Addressed capstone rubric points](#Addressed-capstone-rubric-points)
* [Docker](#Docker)
  * [Requirements](#Requirements)
  * [Build and run](#Build-and-run)
  * [Build and run the project](#Build-and-run-the-project)
* [References](#References)

### Project file and class structure

### Addressed capstone rubric points
The README indicates which rubric points are addressed. The README also indicates where in the code

### Docker
#### Requirements
The following debian installation is required to run the dockers:
```sh
sudo apt-get install python3-rocker
```
#### Build and run
Clone this repository and navigate to docker directory:
```sh
git clone https://github.com/YueErro/udacity_cplusplus.git ~/udacity_cplusplus
cd ~/udacity_cplusplus/Capstone-ROS-Node-simulated-in-Gazebo/docker
```
Build the docker
```sh
bash build.sh
```
Run the docker according to your graphics card (Nvidia or integrated Intel):
```sh
bash run.sh <nvidia/intel>
```

#### Build and run the project
* commands in each and every terminal in the docker
* gif
* expected behavior or output of the program.

### References
* [docker docs](https://docs.docker.com/engine/install/ubuntu/)
* [osrf/rocker](https://github.com/osrf/rocker)