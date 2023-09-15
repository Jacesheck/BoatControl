#!/bin/bash

port=$(head sketch.yaml -n1 | awk '{print $2}')
sudo arduino-cli monitor -p $port
