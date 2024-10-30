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

SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)

source "$SCRPT_DIR_PATH/../../cores_settup.sh"

if [[ -z "${PITON_ROOT}" ]]; then
  PITON_ROOT=$(realpath ${SCRPT_DIR_PATH}/../../../../..)
  echo "PITON_ROOT is set by script to:"
  echo "    $PITON_ROOT"
fi

#my_link=$SCRPT_DIR_PATH/common_ariane/crt.S
#if ! [ -L ${my_link} ] || ! [ -e ${my_link} ] ; then 
#	echo "Settup symlink files"
#	cd $SCRPT_DIR_PATH; bash settup.sh; 
#fi


cores=( "ariane" )
core="ariane"  #default
Dtypes=( "float" "int" "double" )
Dtype="double"
apps=( "mt-axpy" "mt-vadd" "mt-matmul" "mt-spmv" "mt-somier" "mt-histogram" "mt-stream" "mt-stream-copy"  "mt-stream-triad" "mt-int-sort" "mt-emb-parl" "mt-hello") 
app="mt-axpy" #default
mt=1
seed=1
Dump=0 

declare -A commons                              # Or declare separately
commons=( ["ariane"]="../common_ariane" \
) 

declare -A cflags
cflags=(  ["ariane"]="-DARIANE_TILE -march=rv64imafdc -mabi=lp64d" \
) 


declare -A sources
sources=( ["mt-hello"]="mt-hello.c" \
          ["mt-stream"]="mt-stream.c stream.c nop.c" \
)

declare -A default_problems 
default_problems=( \
          ["mt-hello"]="0" \
          ["mt-stream"]="1024" \
);





usage () {
	echo "./compile [options]
      
      [options]
      -h show this help 
      -c <core name>         : ${cores[*]}. Default is ariane 
      -a <application name>  : ${apps[*]}.  Default is mt-axpy
      -m <multi core number> : number of multi cores . Default is one 
      -p <problem size int>  : 
      -s <int number>        : Random seed number using for generating dataset. 
      -d <Data type>         : ${Dtypes[*]}. Default is double.  
      -e <extra arguent>     :  
      -x                     : create hex dump file  
      -S <int number>        : Stack pointer size in B
      "   
       exit 0  
}


extra_arg=""
defines=""
Stk="131072"

while getopts "h?c:a:m:p:s:d:e:xS:D:" opt; do
  case "$opt" in
    h|\?)
      usage     
      ;;
    c) core=$OPTARG
	    if [[ ! " ${cores[*]} " =~ " ${core} " ]]; then
    		echo "${core} is an invalid value for -c option"
    		usage
		fi
      ;; 
    a) app=$OPTARG
      if [[ ! " ${apps[*]} " =~ " ${app} " ]]; then
    		echo "${app} is an invalid value for -a option"
    		usage
	  fi
      ;;
    m) mt=$OPTARG
      ;;
    p) problem=$OPTARG
      ;;  
    s) seed=$OPTARG
      ;; 
    d) Dtype=$OPTARG
      ;;
    x) Dump=1 
    ;;
    S) Stk=$OPTARG
      ;;
    D) 
       defines=$OPTARG
      ;;
    e)extra_arg=$OPTARG
      if [[ ! " ${Dtypes[*]} " =~ " ${Dtype} " ]]; then
    		echo "${Dtype} is an invalid value for -d option"
    		usage
	  fi
      ;;  
   
  esac
done

shift $((OPTIND-1))

[ "${1:-}" = "--" ] && shift


if [[ -z $problem ]]; then
    problem=${default_problems[$app]}    
fi


echo "core=$core, multi_core=$mt app=$app, problem=$problem, seed=$seed, Dtype=$Dtype $extra_arg"


#define variables
#RISCV=~/scratch/`whoami`/riscv_install




common=${commons[$core]}


bin="bin/${core}_${app}${mt}_${problem}.riscv"
source=${sources[$app]}
INCLUDES="-I../ -I${common}/env -I${common}" # -I/usr/include/"
       
FLAGS="-DPREALLOCATE=1 -mcmodel=medany -static -std=gnu99 -O2 -ffast-math -fno-common -fno-builtin-printf"
COMMN=" ${common}/syscalls.c $common/crt.S -static -nostdlib -nostartfiles -lm -lc -lgcc -specs=nano.specs -specs=nosys.specs -T ${common}/test.ld"



CFLAG=${cflags[$core]};

#Step1 enter to app folder
cd $SCRPT_DIR_PATH/$app

mkdir -p  bin
rm -f $bin
rm -f dataset.h

if [[ $core == "coyote" ]]; then
	sed -i '/li a7/c li a7, '$mt ../common_coyote/crt.S   
fi

#step2 generate datase.h
if [[ -f "./gendata.pl" ]]; then
	perl ./gendata.pl --size $problem --seed $seed --Dtype $Dtype $extra_arg > dataset.h
fi



#step3 compile
#set -x




if [ -z ${settups[$core]} ]; then
    echo "Error $core is an unsupported architecture" >&2
fi


bash -c "

echo \"settup for $core : ${settups[$core]} \" 
cd $PITON_ROOT; source ${settups[$core]}; cd $SCRPT_DIR_PATH/$app
compiler_bin=\"\$RISCV/bin/\"


echo \"Generating binary file ....\"
echo \"\$compiler_bin/riscv64-unknown-elf-gcc  $INCLUDES $FLAGS $CFLAG -DPITON_NUMTILES=$mt -DPITONSTREAM -DPITON_STKSIZ=$Stk -o $bin $source $COMMN\"
\$compiler_bin/riscv64-unknown-elf-gcc  $INCLUDES $FLAGS $CFLAG -DPITON_NUMTILES=$mt -DPITONSTREAM -DPITON_STKSIZ=$Stk $defines -o $bin $source $COMMN 


if [[ $Dump == 1 ]]
then
	echo \"Generating Dump file ....\"
	echo \"\$compiler_bin/riscv64-unknown-elf-objdump --disassemble-all --disassemble-zeroes --section=.text.init --section=.text --section=.text.startup --section=.data $bin > $bin.dump\"
	\$compiler_bin/riscv64-unknown-elf-objdump --disassemble-all --disassemble-zeroes --section=.text.init --section=.text --section=.text.startup --section=.data $bin > $bin.dump
fi

"
