#!/bin/sh

usage()
{
cat << EOF
  usage: ./$(basename $0) <nvidia_or_intel>
  This script runs a docker with the graphics
  e.g.: ./run.sh nvidia
  e.g.: ./run.sh intel
EOF
exit 0
}

main()
{
  if [ -z "$1" ]; then
    usage
  elif [ $1 = "nvidia" ]; then
    rocker --nvidia --x11 --user --home udacity-cplusplus
  elif [ $1 = "intel" ]; then
    rocker --x11 --user --home udacity-cplusplus --devices /dev/dri/card0
  else
    usage
  fi
}

main "$@"