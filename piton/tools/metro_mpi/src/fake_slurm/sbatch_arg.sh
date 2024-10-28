#!/bin/bash

#!/bin/bash
SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)
WORK_DIR="$SCRPT_DIR_PATH/.slurm"
PENDING="$WORK_DIR/pending"
RUNNING="$WORK_DIR/running"
LOCK="$WORK_DIR/.lock"
LOCK_FILE="$WORK_DIR/engin.lock"

# Function to remove the lock file
remove_lock_file() {
    rm -f "$LOCK_FILE"
}


#global args
timeout=""
jobname=""
output=""
job_file=""
job_dir=""
job_full=""

# Function to convert time to minutes
convert_time_to_minutes() {
    local time=$1
    local minutes

    if [[ $time == *":"* ]]; then
        # Time is in HH:MM:SS format
        IFS=":" read -r hours minutes seconds <<< "$time"
        minutes=$((hours * 60 + minutes))
    else
        # Time is in minutes
        minutes="$time"
    fi

    echo "$minutes"
}

set_val () {    #(default,val1,val2)
    val=$2
    if [ -z "$val" ]; then
        val="$3"
    fi
    if [ -z "$val" ]; then
        val=$1 #default
    fi
    echo $val
}    

process_sbatch_job_file () {
    in_file="$1"
    local call_dir;
    local file;
    while read -r line; do
        if [ -z "$call_dir" ]; then
            call_dir="$line"
        elif [ -z "$file" ]; then
            file="$line"
            break  # Stop after reading the second line
        fi    
    done <<< "$in_file"    
    
    file_path=$(dirname "$file")
    file_name=$(basename "$file")
    
          
   if [ -e "$call_dir/$file" ]; then
      job_dir="$call_dir/$file_path"
   else
      job_dir="$file_path"
   fi
    job_file=$file_name
    job_full="$job_dir/$job_file"
    if [ ! -f "$job_full" ]; then
        echo "Error: Source file does not exist: $job_full" >&2
        exit 1  # Exit if the source file does not exist
    fi

     
    # Use grep to extract #SBATCH lines from the input file
    SBATCH_lines=$(grep '#SBATCH' "$job_full")
    # Use awk to extract values and store them in variables
    while read -r line; do
        # echo " $line "
        line=$(echo "$line" | sed -E 's/\s+-+/\ /g')   #change --arg or -arg to arg
        line=$(echo "$line" | sed -E 's/\s?=\s?/\ /g') #replace = with whitespace 
        
        var_name=$(echo "$line" | awk -F'[[:space:]]+' '{print $2}')
        var_value=$(echo "$line" | awk -F'[[:space:]]+' '{print $3}')
        var_name=$(echo "$var_name" | sed -E 's/-/\_/g')   #replace - in arg with _ 
        var_name="sbatch_$var_name"  
        #echo "var_name = $var_name var_value=$var_value"
        # Assign values to variables
        eval "$var_name=\"$var_value\""
        
    done <<< "$SBATCH_lines"


timeout_raw=$(set_val "2:0:0" $sbatch_t $sbatch_time )
# Convert the --time argument to minutes
timeout=$(convert_time_to_minutes "$timeout_raw")

jobname=$(set_val "123$MY_PID" $sbatch_J $sbatch_job_name )



output=""
if [ ! -z "$sbatch_output" ]; then
    output=" > ./$sbatch_output "
fi
if [ ! -z "$sbatch_error" ]; then
    output="$output 2> ./$sbatch_error "
fi

}


jobs=()
get_file_list () {
    local dir_in=$1   
    if [ ! -d "$dir_in" ]; then
        echo "Error: Source is not a directory: $dir_in" >&2
        exit 1  # Exit if source is not a directory
    fi 
    jobs=()
    # List files, get their creation times, and sort by creation time
    get_exclusive_access
    files_with_creation_time=$(find "$dir_in" -type f -exec stat -c "%Y %n" {} + | sort -n)
    unlock
    # Check if the directory is empty
    if [ -z "$files_with_creation_time" ]; then       
        return
    fi    
    # Extract and display the sorted list
    while IFS= read -r line; do
        creation_time=$(echo "$line" | cut -d ' ' -f 1)
        local file_name1=$(echo "$line" | cut -d ' ' -f 2)
        local jobid1=$(basename "$file_name1")
        jobs+=("$jobid1")
    done <<< "$files_with_creation_time"   
}


get_exclusive_access () {
    exec 100>$LOCK || exit 1
    flock -w 200 100 || exit 1
}

unlock () {
    flock -u 100 || exit 1
}




#process_sbatch_job_file "$(cat "/home/alireza/work/git/openpiton/Perte2/piton/tools/metro_mpi/src/fake_slurm/.slurm/pending/1" ) "


