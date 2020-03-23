#!/bin/bash

rosservice call /param_get X_EQ_TORQUE && \
rosservice call /param_get Y_EQ_TORQUE && \
rosservice call /param_get Z_EQ_TORQUE
