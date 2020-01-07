#!/bin/bash

s_name='sim_ws'

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
source "$DIR/devel/setup.bash"

echo -e "\e[1m$s_name workspace sourced\e[0m"

GREEN="\[\033[32m\]"
RESET="\[$(tput sgr0)\]"
rosenv="${GREEN}($s_name)"

if [ "$SOURCED_WS" == 'TRUE' ]
then
    echo -e "\e[36mprevious sourcing overwritten\e[0m"
else
    export PS1="${PS1}${rosenv} ${RESET}"
    export SOURCED_WS='TRUE'
fi
alias ros_make="(cd `echo $ROS_PACKAGE_PATH | awk -Fsrc: '{print $1}'` && catkin_make)"

# ADD CUSTOM GAZEBO PATHS HERE



# ADD OTHER PROJECT ENVIRONMENT VARIABLES HERE
