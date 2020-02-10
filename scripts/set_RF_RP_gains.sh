#!/bin/bash

GAINTYPE=$1
GAINVALU=$2

rosservice call /param_set "PID_ROLL_ANG_$GAINTYPE" $GAINVALU && \
rosservice call /param_set "PID_PITCH_ANG_$GAINTYPE" $GAINVALU
