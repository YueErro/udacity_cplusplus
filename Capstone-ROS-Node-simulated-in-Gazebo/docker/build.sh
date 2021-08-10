#!/bin/sh

docker build --build-arg USER=$USER --build-arg IMAGE_NAME="udacity-cplusplus" -t udacity-cplusplus .
