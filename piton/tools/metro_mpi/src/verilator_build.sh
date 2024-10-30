#!/bin/bash

# Copyright (c) 2024, Barcelona Supercomputing Center
# Contact: alireza.monemi [at] bsc [dot] es
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the copyright holder nor the names
#       of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#VERILATOR_VERSION=4.228
#VERILATOR_VERSION=4.104
#VERILATOR_VERSION=4.224
#VERILATOR_VERSION=4.204
#VERILATOR_VERSION=5.012
#VERILATOR_VERSION=5.004
#VERILATOR_VERSION=5.010

VERILATOR_VERSION=4.104  #The default version. u can change it with -v flag

download_only=0
install_only=0
NUM_JOBS=4

help(){
      echo "./$0 [options]
      
      [options]
      -h show this help 
      -d            : Only download the verilator repo
      -i            : Only install the verilator.
      -j <num>    : number of parallel jobs passed to makefile. default is 4
      -v <version>: Verilatot version number. Default is $VERILATOR_VERSION
      
      If none of -d or -i are given, the script performs both of the download and installaton steps. 
    "      
      exit 0;
}


#process input arguments
while getopts "h?dij:v:" opt; do
  case "$opt" in
    h|\?)
      help
      exit 0
      ;;
    d) download_only=1
      ;; 
    i) install_only=1
      ;;   
    j) NUM_JOBS=$OPTARG
      ;;
    v) VERILATOR_VERSION=$OPTARG      
  esac
done
shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift


#if none of only flag is given download & install verilator
if [ $download_only == 0 ] && [ $install_only == 0 ] ; then 
    download_only=1
    install_only=1
fi



Vname=${VERILATOR_VERSION//./_}
Vcheckout=v$VERILATOR_VERSION

dir=~/scratch/`whoami`/verilator_repo
instal_dir=~/scratch/`whoami`/verilator_$Vname

if [ $download_only == 1 ]; then 
    echo
    echo "----------------------------------------------------------------------"
    echo "Downloading Verilator toolchain $VERILATOR_VERSION"
    echo "----------------------------------------------------------------------"
    echo
    
    mkdir -p $dir
    cd $dir
    [ -d verilator ] || git clone https://github.com/verilator/verilator
    cd verilator
    unset VERILATOR_ROOT # For bash
    #git checkout stable # Use most recent stable release
    git fetch
    git checkout $Vcheckout
    if [ $? -eq 0 ]; then
            echo "Git checkout was successful."
            # Additional commands to execute if checkout was successful
    else
            echo "Git checkout $Vcheckout failed. There might be an error."
            exit 1;
    fi    
    
    echo
    echo "----------------------------------------------------------------------"
    echo "Download complete"
    echo "----------------------------------------------------------------------"
    echo
    
fi

if [ $install_only == 1 ]; then 
    echo
    echo "----------------------------------------------------------------------"
    echo "building Verilator toolchain $VERILATOR_VERSION"
    echo "----------------------------------------------------------------------"
    echo

    # Check the verilator repo heading the correct version as the given $VERILATOR_VERSION
    # Extract the version from CMakeLists.txt
    cd $dir/verilator
    repo_version=$(grep -oP 'Verilator\s+\K\d+(\.\d+)*' Changes | head -n 1)
    if [[ "$repo_version" != "$VERILATOR_VERSION" ]]; then
    echo "Versions do not match. Extracted repo version: $repo_version, Given version: $VERILATOR_VERSION. There might be an error."
        exit 1;
    fi    

    mkdir -p $instal_dir
    unset VERILATOR_ROOT
    autoconf
    echo "./configure --prefix $instal_dir"
    ./configure --prefix $instal_dir
    
    make clean
    rm -f $dir/verilator/bin/verilator_bin 
    rm -f $dir/verilator/bin/verilator_bin_dbg
    rm -f $dir/verilator_coverage_bin_dbg
    make -j 2 #NUM_JOBS #does not work for large number? 
    make install
    cp -R $dir/verilator/*    $instal_dir/
    
    #add #include <limits> to the beginning of the $instal_dir/include/verilated.cpp file.
    file="$instal_dir/include/verilated.cpp"
    # Check if the file exists
    if [ -f "$file" ]; then
      # Add "#include <limits>" to the beginning of the file if it's not already present
      if ! grep -Fxq "#include <limits>" "$file"; then
        sed -i '1i #include <limits>\n' "$file"
        echo "'#include <limits>' has been added to the beginning of $file."
      else
        echo "'#include <limits>' is already present in $file"
      fi
    else
      echo "File $file does not exist!"
    fi
    
    echo
    echo "----------------------------------------------------------------------"
    echo "build complete"
    echo "----------------------------------------------------------------------"
    echo

fi
