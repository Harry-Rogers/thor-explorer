#!/usr/bin/env bash
rosrun map_server map_saver -f map
mv map.pgm map.yaml ~/thor-explorer/thor-explorer/src/thor-explorer/map/
