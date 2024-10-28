#!/bin/bash

SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)

if [[ -z "${PITON_ROOT}" ]]; then
  PITON_ROOT=$(realpath ${SCRPT_DIR_PATH}/../../..)
fi

source ${SCRPT_DIR_PATH}/cores_settup.sh

mpi_bin=""
if  command -v mpic++ &> /dev/null
then
   mpi_bin=mpic++  
elif command -v mpicxx &> /dev/null
then
   mpi_bin=mpicxx   
else
    echo "Error: None of mpic++ or mpicxx could be found! Could not continue."
    exit
fi


work="$PITON_ROOT/build"

x_tiles="1"
y_tiles="1"
app="hello_world_token.c"
core="ariane"

build_chipset=0
build_metro_tile=0
build_fake_mem=0
run_sim=0
pronoc=""
multimc=""
hbm_lat=""
rtl_flag="-config_rtl=PITON_CUSTOM_IO_INIT_BASE"
extra_args=""
build_locally=0
pronoc_params=""
ovs=""

while getopts "h?x:y:z:a:w:ctfrp:om:l:de:b" opt; do
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
      -c               : build chipset 
      -t               : build tile 
      -f               : build fake_mem                     
      -r               : run sim
      -p <pronoc_param>: enable ProNoCc. Provide ProNoC parameters seprated by coma
      -m <mc pos aray> : Enter the tile numbers where each MC should be connected seprated by ,
      -o               : run mpirun with --oversubscribe flag. Default is not implimented.
                         Nodes are allowed to be oversubscribed, even on a managed system, 
                         and overloading of processing elements
      -l  <clk number> : Enable HBM RD latency
      -d               : Disable all monitors
      -e               : extra arguments
      -b               : run build command locally
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
    c) build_chipset=1
      ;;  
    t) build_metro_tile=1
      ;; 
    f) build_fake_mem=1
      ;;
    r) run_sim=1
      ;;   
    w) work=$OPTARG
      ;;
    p) pronoc="-pronoc"
       pronoc_params="-pronoc_params  \"$OPTARG\""
      ;;         
    o) ovs="export MPI_OVERSUBSCRIBE=1" 
      ;;  
    l) hbm_lat="-hbm_lat=$OPTARG"
      ;;
    d) rtl_flag="$rtl_flag -config_rtl=DISABLE_ALL_MONITORS -config_rtl=MINIMAL_MONITORING"
      ;;
    e) extra_args=$OPTARG
      ;;
    b)
      build_locally=1
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
echo "	build_chipse=$build_chipset"
echo "	build_metro_tile=$build_metro_tile"
echo "	build_fake_mem = $build_fake_mem"
echo "	run_sim=$run_sim"
echo "	pronoc=$pronoc $pronoc_params"
echo "	work=$work"
echo "	multimc=$multimc"
echo "	rtl_flag=$rtl_flag"
echo "	extra_args=$extra_args"
echo "  build_locally=$build_locally"



flag="-metro_mpi_build_args=\"-CFLAGS -O2 \"  "


cd $PITON_ROOT; mkdir -p $work
cd $work



  
if [ -z ${settups[$core]} ]; then
    echo "Error $core is an unsupported architecture" >&2
    
fi

# Necessary source scripts for OpenPiton
bash -c " 
$ovs
echo \"settup for $core : ${settups[$core]} \" 
cd $PITON_ROOT; source ${settups[$core]}; cd $work

function build_func () {
   sys=\$1
   if [ $build_locally = 0 ] ; then
     sims -metro_mpi_build -metro_mpi_build_config=manycore -$core -metro_mpi_build_args=\"-MAKEFLAGS CXX=$mpi_bin -MAKEFLAGS LINK=$mpi_bin\" -metro_mpi_build_args=\"-j 2 \" $flag $rtl_flag -sys=\$sys -x_tiles=$x_tiles -y_tiles=$y_tiles -model_dir=$PWD $pronoc $pronoc_params $multimc $hbm_lat $extra_args

     cp $SCRPT_DIR_PATH/src/make_local.pl ./\$sys/rel-0.1
     cd ./\$sys/rel-0.1
     perl make_local.pl
     # Check if the Perl script exited with an error
     if [ \$? -ne 0 ]; then
        echo \"Perl make_local.pl script exited with an error.\"
        exit 1
     fi
     cd -
  fi
  if [ $build_locally = 1 ] ; then
    cd ./\$sys/rel-0.1; bash build_local.sh; cd -
  fi
   
}



# Build the metro_chipset (with -Os roughly 5 min)
if [ $build_chipset = 1  ] ;  then
  build_func   \"metro_chipset\"
fi


if [ $build_fake_mem = 1  ]
then
   build_func \"metro_fake_mem\"  
fi

# Build the metro_tile 
if [ $build_metro_tile = 1 ]  ; then
   build_func \"metro_tile\" 
fi

if [ $run_sim = 1 ]  
then
  #remove old results
  rm  -f fake_uart.log  sims.log
  # Run the model 
  sims -metro_mpi_run -$core -metro_mpi_build_config=manycore -sys=metro_chipset  -x_tiles=$x_tiles -y_tiles=$y_tiles $app -model_dir=$PWD $pronoc $pronoc_params $multimc $hbm_lat $extra_args
fi

"

