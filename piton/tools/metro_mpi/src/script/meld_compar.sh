#!/bin/bash
SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)
# Default values for options
num_lines=1000   # Default for -n
start_line1=1    # Default for -s
start_line2=1    # Default for -t

# Parse the options using getopts
while getopts "n:s:t:" opt; do
    case $opt in
        n)
            num_lines="$OPTARG"
            ;;
        s)
            start_line1="$OPTARG"
            ;;
        t)
            start_line2="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

# Shift positional arguments to access file1 and file2
shift $((OPTIND-1))

# Check if the positional arguments are provided
if [ $# -lt 2 ]; then
    echo "Usage: $0  -n num_lines -s start_line1 -t start_line2  file1 file2"
    exit 1
fi



file1=$1
file2=$2

# Output the parsed values (for debugging purposes)
echo "File 1: $file1"
echo "File 2: $file2"
echo "Number of lines: $num_lines"
echo "Start line for file1: $start_line1"
echo "Start line for file2: $start_line2"


# Function to extract lines from a file
extract_lines() {
    local input_file=$1
    local start_line=$2
    local num_lines=$3
    local output_file=$4
    # Check if input file exists
    if [ ! -f "$input_file" ]; then
        echo "Error: Input file '$input_file' not found."
        exit 1
    fi

    # Use sed to extract the lines
    sed -n "${start_line},$((${start_line}+${num_lines}-1))p" "$input_file" > "$output_file"
    
    if [ ! -f "$output_file" ]; then
        echo "Error: Failed to generate output file '$output_file'."
        exit 1
    fi
}


extract_lines $file1 $start_line1 $num_lines "$file1.tmp"
extract_lines $file2 $start_line2 $num_lines "$file2.tmp"


meld $file1.tmp $file2.tmp


