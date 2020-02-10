#!/bin/bash

PID_ROLL_ANG_P=`rosservice call /param_get PID_ROLL_ANG_P | grep -Eo "[0-9]+\.[0-9]+"`
PID_ROLL_ANG_I=`rosservice call /param_get PID_ROLL_ANG_I | grep -Eo "[0-9]+\.[0-9]+"`
PID_ROLL_ANG_D=`rosservice call /param_get PID_ROLL_ANG_D | grep -Eo "[0-9]+\.[0-9]+"`

PID_PITCH_ANG_P=`rosservice call /param_get PID_PITCH_ANG_P | grep -Eo "[0-9]+\.[0-9]+"`
PID_PITCH_ANG_I=`rosservice call /param_get PID_PITCH_ANG_I | grep -Eo "[0-9]+\.[0-9]+"`
PID_PITCH_ANG_D=`rosservice call /param_get PID_PITCH_ANG_D | grep -Eo "[0-9]+\.[0-9]+"`

PID_YAW_RATE_P=`rosservice call /param_get PID_YAW_RATE_P | grep -Eo "[0-9]+\.[0-9]+"`
PID_YAW_RATE_I=`rosservice call /param_get PID_YAW_RATE_I | grep -Eo "[0-9]+\.[0-9]+"`
PID_YAW_RATE_D=`rosservice call /param_get PID_YAW_RATE_D | grep -Eo "[0-9]+\.[0-9]+"`

printf "ROLL PID:\t%.3f\t%.3f\t%.3f\n" "$PID_ROLL_ANG_P" "$PID_ROLL_ANG_I" "$PID_ROLL_ANG_D"
printf "PITCH PID:\t%.3f\t%.3f\t%.3f\n" "$PID_PITCH_ANG_P" "$PID_PITCH_ANG_I" "$PID_PITCH_ANG_D"
printf "YAWRATE PID:\t%.3f\t%.3f\t%.3f\n" "$PID_YAW_RATE_P" "$PID_YAW_RATE_I" "$PID_YAW_RATE_D"
