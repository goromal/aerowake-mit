#!/bin/bash

FBPARG="$1"
OUTBAGNAME="$2"
T0="$3"
TF="$4"

if [[ "$FBPARG" = /* ]]; then
  FULLBAGPATH="$FBPARG"
else
  FULLBAGPATH="$PWD/$FBPARG"
fi

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

BAGDIR=$(dirname "$FULLBAGPATH")
BAGFILENAME=$(basename -- "$FULLBAGPATH")
BAGNAME="${BAGFILENAME%.*}"

if [[ -z $T0 || -z $TF ]]; then
  echo "Time endpoints either not or improperly specified. Moving on..."
else
  echo "Performing time-based filtering: $T0 <= t <= $TF..."
  RM0="$BAGDIR/$BAGNAME-tfilter-$T0-$TF.bag"
  rosbag filter "$FULLBAGPATH" "$RM0" "t.secs >= $T0 and t.secs <= $TF"
  FULLBAGPATH="$RM0"
  BAGDIR=$(dirname "$FULLBAGPATH")
  BAGFILENAME=$(basename -- "$FULLBAGPATH")
  BAGNAME="${BAGFILENAME%.*}"
fi

echo "Removing irrelevant topics..."
RM1="$BAGDIR/$BAGNAME-topicfilter.bag"
rosbag filter "$FULLBAGPATH" "$RM1" "topic == '/rc_raw' or topic == '/odometry'"
FULLBAGPATH="$RM1"
BAGDIR=$(dirname "$FULLBAGPATH")
BAGFILENAME=$(basename -- "$FULLBAGPATH")
BAGNAME="${BAGFILENAME%.*}"

echo "Renaming remaining topics..."
"$DIR/rename_bag_topics.py" "$FULLBAGPATH" "/rc_raw" "/RC" "/odometry" "/hw_odometry"
RM2="$BAGDIR/$BAGNAME-RENAMED.bag"
FULLBAGPATH="$RM2"
BAGDIR=$(dirname "$FULLBAGPATH")
BAGFILENAME=$(basename -- "$FULLBAGPATH")
BAGNAME="${BAGFILENAME%.*}"

echo "Compressing bag..."
rosbag compress "$FULLBAGPATH"
mv "$FULLBAGPATH" "$BAGDIR/$OUTBAGNAME.bag"

echo "Cleaning up..."
if [[ !( -z $RM0 ) ]]; then
  rm "$RM0"
fi
rm "$RM1"
rm "$BAGDIR/$BAGNAME.orig.bag"
