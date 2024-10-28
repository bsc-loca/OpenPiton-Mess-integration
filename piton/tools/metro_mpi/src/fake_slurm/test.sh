#!/bin/bash

SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)

bash $SCRPT_DIR_PATH/sbatch $SCRPT_DIR_PATH/tmp2.sh &
bash $SCRPT_DIR_PATH/sbatch $SCRPT_DIR_PATH/tmp2.sh &
bash $SCRPT_DIR_PATH/sbatch $SCRPT_DIR_PATH/tmp2.sh &

wait
