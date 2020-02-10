#!/bin/bash

PARAMFILENAME="rosflight_params_aerowake-no-rc_Jan2020.yaml"

##################################################
# GET DIRECTORY OF THIS SCRIPT TO ACCESS RESOURCES
##################################################

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

#########
# EXECUTE
#########

source "$DIR/../gs_ws/sourceror.sh"
PARAMPATH=`rospack find aerowake_params`
rosservice call /param_load_from_file "$PARAMPATH/params/$PARAMFILENAME"
sleep 5
rosservice call /param_write
