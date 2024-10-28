#!/bin/bash



#SBATCH --job-name="build-Metro-MPI-model110"
#SBATCH --output=r.out
#SBATCH --error=r.err
#SBATCH --nodes=2
#SBATCH --cpus-per-task=1
#SBATCH --ntasks=71
#SBATCH --tasks-per-node=48
#SBATCH --qos=debug
#SBATCH --time=0:01:50

mkdir -p ./out;
#time bash ./build.sh > ./out/build-Metro-MPI-model110_log

sleep 50000

echo "done!"
