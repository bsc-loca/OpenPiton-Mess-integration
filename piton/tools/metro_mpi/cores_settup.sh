#!/bin/bash

declare -A settups                              
settups=( ["ariane"]="piton/ariane_setup.sh" \
          ["ost1"]="piton/ariane_setup.sh" \
          ["lite"]="piton/ariane_setup.sh" \
          ["sarg"]="piton/sargantana_setup.sh" \
          ["lox"]="piton/lox_setup.sh" \
) 


# Declare an indexed array to store the unique values
declare -a mySetups=()

# Loop through the associative array values
for setup in "${settups[@]}"; do
    # Check if the value already exists in 'mySetups'
    if [[ ! " ${mySetups[@]} " =~ " ${setup} " ]]; then
        # If not, add it to 'mySetups'
        mySetups+=("$setup")
    fi
done
  

