#!/usr/bin/env bash
rosrun map_server map_saver -f map
mv map.pgm map.yaml ..
cd ..
mv map.pgm map.yaml map
