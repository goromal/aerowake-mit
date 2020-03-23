#!/bin/bash

LOGNAME="roof2_3"
# BAGNAME="ekf_data_collection"
BAGNAME="roof2_3"

MATLAB='/usr/local/MATLAB/R2018b/bin/matlab'

source ../sim_ws/sourceror.sh

roslaunch aerowake_sim process_ekf_data.launch log_ekf:=true bagfile:=$BAGNAME logname:=$LOGNAME redo_vision:=true

$MATLAB -nodisplay -r "process_ekf_logs('$LOGNAME'); quit"

mf_process -p "../bags/ekf_bags/${LOGNAME}_logs/ABS_STATE.fig" "../bags/ekf_bags/${LOGNAME}_logs/ABS_UPDATES.fig" "../bags/ekf_bags/${LOGNAME}_logs/REL_STATE.fig" "../bags/ekf_bags/${LOGNAME}_logs/REL_UPDATES.fig" 

pdfunite ABS_STATE.pdf ABS_UPDATES.pdf REL_STATE.pdf REL_UPDATES.pdf "ekf_figs/${LOGNAME}_ekf_report.pdf"

rm ABS_STATE.pdf
rm ABS_UPDATES.pdf
rm REL_STATE.pdf
rm REL_UPDATES.pdf

evince "ekf_figs/${LOGNAME}_ekf_report.pdf" &
