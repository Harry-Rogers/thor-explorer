#!/usr/bin/env bash
rosrun map_server map_saver -f map
mv areamap.pgm areamap.yaml ..
cd ..
mv areamap.pgm areamap.yaml map
