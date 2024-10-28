#!/bin/bash
SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)

source "$SCRPT_DIR_PATH/sbatch_arg.sh"
source "$SCRPT_DIR_PATH/config"

# Check if the script is running
engin=$(ps aux | grep "slurm_engin.sh" | grep -v grep)

if [ ! -z "$engin" ] ; then
    echo "The script is running. Now it is killed!"
    pkill -f "slurm_engin.sh"
   
fi    
sleep 2
remove_lock_file
rm -f $LOCK
