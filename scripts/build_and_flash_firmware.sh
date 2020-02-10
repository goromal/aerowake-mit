#!/bin/bash

# GET SCRIPT LOCATION
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

# HELPER FUNCTIONS
function decompose_echo_cmd() {
  COLORNUM=$1; shift; FLAGS='-e'; POSITIONAL=()
  while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
      -n)
      FLAGS="${FLAGS} -n"
      shift
      ;;
      -e)
      FLAGS="${FLAGS}"
      shift
      ;;
      -en|-ne)
      FLAGS="${FLAGS} -n"
      shift
      ;;
      *)
      POSITIONAL+=("$1")
      shift
      ;;
    esac
  done
  set -- "${POSITIONAL[@]}"; COFF="\033[0m"
  ECHOSTR="echo ${FLAGS} \"\033[1;${COLORNUM}m$@${COFF}\";"
  eval "${ECHOSTR}"
}
function echo_r() { decompose_echo_cmd 31 $@; }
function echo_g() { decompose_echo_cmd 32 $@; }
function echo_y() { decompose_echo_cmd 33 $@; }
function echo_b() { decompose_echo_cmd 34 $@; }
function check_ports() {
  for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
      syspath="${sysdevpath%/dev}"
      devname="$(udevadm info -q name -p $syspath)"
      [[ "$devname" == "bus/"* ]] && continue
      eval "$(udevadm info -q property --export -p $syspath)"
      [[ -z "$ID_SERIAL" ]] && continue
      echo "/dev/$devname - $ID_SERIAL"
    )
  done
}
function found_rosflight() {
  ROSFLIGHT_PORT=`check_ports | grep "JAMES" | awk -F"| - JAMES" '{print $1}'`
  [[ -z "${ROSFLIGHT_PORT}" ]] && return 0 || return 1
}
function found_flashable() {
  DFUABLE=`dfu-util -l | grep "Found DFU:"`
  [[ -z "${DFUABLE}" ]] && return 0 || return 1
}
function RF_found_error() {
  echo_r "ROSflight board found, which means you didn't short the pins correctly. Please try again..."
  exit
}
function F_not_found_error() {
  echo_r "No flashable board found. Is anything plugged in?"
  exit
}

# SCRIPT EXECUTION

found_rosflight && echo_g "Checking for flashable board..." || RF_found_error
found_flashable && F_not_found_error || echo_g "Flashable board found! Building and flashing..."

cd "$SCRIPTDIR/../firmware/rosflight_firmware"
GITBRANCH=`git rev-parse --abbrev-ref HEAD`
echo_y "Building firmware branch: $GITBRANCH"
sleep 2
make BOARD=REVO flash
