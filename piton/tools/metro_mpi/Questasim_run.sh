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

if [[ -z "${PITON_ROOT}" ]]; then
  PITON_ROOT=$(realpath ${SCRPT_DIR_PATH}/../../..)
fi

source ${SCRPT_DIR_PATH}/cores_settup.sh



export PATH=$PATH:$MODELSIM_BIN;

work="$PITON_ROOT/build"

x_tiles="1"
y_tiles="1"
app="hello_world_token.c"
core="ariane"

build=0
run_sim=0
pronoc=""
run_args=""
multimc=""
mgui_build=""
mgui_run=""
hbm_lat=""
rtl_flag="-config_rtl=PITON_NO_CHIP_BRIDGE  -config_rtl=PITON_CUSTOM_IO_INIT_BASE"
#rtl_flag="$rtl_flag  -config_rtl=PITON_NOC_MONITOR"   
extra_args=""
pronoc_params=""
rtl_timeout="-rtl_timeout=500000"

while getopts "h?x:y:z:a:w:brp:gm:l:de:t:" opt; do
  case "$opt" in
    h|\?)
      echo "./$0 [options]
      
      [options]
      -h show this help 
      -x <int number>  : Enter the number of tiles in x. default is 1
      -y <int number>  : Enter the number of tiles in y. default is 1
      -z <string>      : Enter the core name. Default is $core
      -a <string>      : Enter the app name. Default is $app 
      -w <string>      : work dir. default is $work
      -b               : build the model
      -r               : run sim
      -p <pronoc_param>: enable ProNoCc. Provide ProNoC parameters seprated by coma
      -m <mc pos aray> : Enter the tile numbers where each MC should be connected seprated by ,
      -g               : Enable manyGui (traffic Visualizer)
      -l <#clk/model>  : Enable HBM RD latency clk  number for fix latency, Model path for realstic
      -d               : Minimal monitoring
      -e               : extra arguments
      -t <#clk>        : RTL timeout
"
          
      	  
      
      
      exit 0
      ;;
    x) x_tiles=$OPTARG
      ;; 
    y) y_tiles=$OPTARG
      ;;  
    z) core=$OPTARG
      ;;  
    a) app=$OPTARG
      ;; 
    b) build=1
      ;;  
    r) run_sim=1
      ;;   
    w) work=$OPTARG
      ;;
    p) pronoc="-pronoc"
       pronoc_params="-pronoc_params  \"$OPTARG\""
      ;;  
    g) mgui_build="-mgui_build"
       mgui_run="-mgui_run"
      ;;
    l) hbm_lat="-hbm_lat=$OPTARG"
      ;;
    d) rtl_flag="$rtl_flag  -config_rtl=MINIMAL_MONITORING"
      ;;
    e) extra_args=$OPTARG
      ;;
    t) rtl_timeout="-rtl_timeout=$OPTARG"
      ;;
    m) 
     res="${OPTARG//[^,]}"
	   n="${#res}"
	   n=$(($n+1))
	   multimc="-multimc=$n   -multimc_indices=$OPTARG -hbm"
     if [[ $OPTARG == *"acme"* ]] ; then        
        multimc="-multimc=acme -hbm"
     fi
      ;;
  esac
done




shift $((OPTIND-1))

[ "${1:-}" = "--" ] && shift

echo "	x_tiles=$x_tiles"
echo "	y_tiles=$y_tiles"
echo "	core=$core"
echo "	app=$app"
echo "	build=$build"
echo "	run_sim=$run_sim"
echo "	pronoc=$pronoc $pronoc_params"
echo "	work=$work"
echo "	multimc=$multimc"
echo "	hbm_lat=$hbm_lat"
echo "	rtl_flag=$rtl_flag"
echo "	extra_args=$extra_args"



flag=""


cd $PITON_ROOT; mkdir -p $work
cd $work




if [ -z ${settups[$core]} ]; then
    echo "Error $core is an unsupported architecture" >&2
fi

# Necessary source scripts for OpenPiton
bash -c " 

echo \"settup for $core : ${settups[$core]} \" 
cd $PITON_ROOT; source ${settups[$core]}; cd $work
   

if [ $build = 1 ]  
 then
sims -msm_build -$core $flag $rtl_flag -sys=manycore -x_tiles=$x_tiles -y_tiles=$y_tiles -model_dir=$PWD $pronoc  $pronoc_params $multimc $mgui_build $hbm_lat $extra_args $rtl_timeout -sim_run_args=\"-64\"
fi



if [ $run_sim = 1 ]  
 then
# Run the model 
sims -msm_run -$core -sys=manycore  -x_tiles=$x_tiles -y_tiles=$y_tiles $app -model_dir=$PWD $pronoc $pronoc_params $run_args -gui $multimc $mgui_run $hbm_lat $extra_args $rtl_timeout -sim_run_args=\"-64\"
fi

"

