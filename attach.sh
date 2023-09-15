#!/bin/bash

port=$(arduino-cli board list | grep Arduino | awk '{print $1}')

sudo arduino-cli board attach -p $port
