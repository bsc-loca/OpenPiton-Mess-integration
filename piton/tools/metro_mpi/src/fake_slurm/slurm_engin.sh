#!/bin/bash



SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)
WORK_DIR="$SCRPT_DIR_PATH/.slurm"



source "$SCRPT_DIR_PATH/sbatch_arg.sh"
source "$SCRPT_DIR_PATH/config"

#make job dirs if not existed
mkdir -p $PENDING
mkdir -p $RUNNING

# Trap signals to remove the lock file on script interruption
trap 'remove_lock_file; exit 1' INT TERM

# Check if the script is running
# Check if the lock file exists
if [ -e "$LOCK_FILE" ]; then
    echo "Script is already running. Exiting." >&2
    exit 1
fi
# Create the lock file
echo "$$" > "$LOCK_FILE"


rm -rf $RUNNING/*  #make sure the running folder is empy at start time


# Declare an associative array
declare -A run_pids
run_pending_job () {
    to_run=$1
    get_file_list $PENDING    
    #sleep 5  #delay to make sure all captured pending files are updated
    for ((i=0; i<to_run; i++)); do
        local job_name2=${jobs[$i]}
        get_exclusive_access
        jj="$(cat "$PENDING/$job_name2")"
        unlock
        
        process_sbatch_job_file  "$jj"
       
        get_exclusive_access
        if [ -d "$PENDING/$job_name2" ]; then
            echo "Error: $PENDING/$job_name2 is a directory" >&2
            unlock
            exit 1  # Exit if source is a directory
        fi
        mv $PENDING/$job_name2 $RUNNING/$job_name2
        touch $RUNNING/$job_name2
        unlock
        cmd="cd $job_dir; timeout ${timeout}m bash ./$job_file $output &"
        eval "$cmd"        
        timeout_pid=$!
        run_pids["$job_name2"]="$timeout_pid"
        current_time=$(date +"%T")
        echo "Started $current_time  ${jobs[$i]}      $timeout_pid  $> $cmd "
    done
}

check_runnig_job () {
    local  job_name3=$1
    local job_pid=$2     
    if ! ps -p $job_pid > /dev/null
    then
        current_time=$(date +"%T")
        echo "Ended   $current_time  $job_name3      $job_pid  "
        get_exclusive_access
        rm $RUNNING/$job_name3
        unlock
        unset run_pids["$job_name3"]
     fi 
 }


#main loop 
run=1
num=0
sleep $SLURM_CHECK_PERIOD

echo "Status  Time      Sbatch_Job_ID PID    Exec command"

while [ $run -eq 1 ]; do  
    
    #1 check running jobs and if they are finished remove them from running list   
    for job_name1 in "${!run_pids[@]}"; do
        job_pid="${run_pids[$job_name1]}"
        check_runnig_job $job_name1 $job_pid
    done    
    
    #2 get the number of running jobs if its smaler than MAX_RUN pick a pending job 
    run_num=$(find "$RUNNING" -type f | wc -l)  
    pending_num=$(find "$PENDING" -type f | wc -l)


    if [ $run_num -lt $MAX_RUN ]; then 
        can_run=$((MAX_RUN - run_num < pending_num ? MAX_RUN - run_num : pending_num))
        if [ $can_run -gt 0 ]; then 
                run_pending_job $can_run
                num=0
        fi        
    fi 
    
    num=$((run_num + pending_num > 0 ? 0 : num + 1))
    if [ $num -gt 10 ]; then #if there is no activity for 100 second close the engine
        run=0
        echo "all jobs are done. Exit the engin!"
    fi 
    sleep $SLURM_CHECK_PERIOD #  check the jobs status every 10 second 
   
done


# Remove the lock file when the script finishes
remove_lock_file

