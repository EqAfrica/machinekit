#!/bin/bash
if [ -z "$EMC2_HOME" ]; then
  # . ~/projekt_afrika/emc2-dev/scripts/rip-environment
  . ~/projekt_afrika/machinekit/scripts/rip-environment
fi
export LANG=en_US.UTF-8
export LANGUAGE=en_US.UTF-8
# export LANG=zh_TW.utf8
# export LANGUAGE=zh_TW.utf8
cd $EMC2_HOME/src
