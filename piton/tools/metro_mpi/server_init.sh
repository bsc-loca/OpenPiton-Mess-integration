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

VERILATOR_VERSION=4.104  #The default version. u can change it with -v flag
MODULES=""
SCRPT_FULL_PATH=$(realpath ${BASH_SOURCE[0]})
SCRPT_DIR_PATH=$(dirname $SCRPT_FULL_PATH)

source ${SCRPT_DIR_PATH}/cores_settup.sh

if [[ -z "${PITON_ROOT}" ]]; then
  PITON_ROOT=$(realpath ${SCRPT_DIR_PATH}/../../..)
fi


NUM_JOBS=16
copy_meep_repo=0
copy_riscv_repo=1
install_rsicv_server=1
copy_verilator_repo=1
install_csh=0
install_ccache=0
CONTINT=0

install_verilator_server=1
copy_dtc_server=1

add_perl_to_path=0
add_csh_to_path=0
add_ccache_to_path=0
enable_checkpoints=0
install_perl=0
add_sbatch_to_path=0
install_sbatch=0
add_help2man_to_path=0
install_help2man=0     

VERILATOR_EXLUDE="
--exclude=verilator/bin/verilator_bin \
--exclude=verilator/bin/verilator_bin_dbg \
--exclude=verilator_coverage_bin_dbg \
--exclude=*.o" 


EXCLUDES="
--exclude=*.tmp.v \
--exclude=*.tmp.sv \
--exclude=*.tmp.h \
--exclude=*.pdf \
--exclude=*.dump \
--exclude=*.wlf \
--exclude=*.jpg \
--exclude=*.JPG \
--exclude=*.o \
--exclude=design/chip/tile/ariane/corev_apu/fpga-support/synth \
--exclude=design/chip/tile/ariane_hpdc/corev_apu/fpga-support/synth \
--exclude=design/aws/* \
--exclude=design/chip/chip_bridge/xilinx/* \
--exclude=design1/chip/tile/ariane/core/fpu/tb/* \
--exclude=design1/chip/tile/ariane/corev_apu/tb/* \
--exclude=tools/metro_mpi/src/riscv_install.tar.xz \
--exclude=tools/metro_mpi/benchmark/app-scalar/*/dataset.h \
--exclude=tools/metro_mpi/benchmark/app-scalar/*/bin/* \
--exclude=verif1/diag/assembly/* \
--exclude=*/.git/" 



uname=""

help(){

echo "./$0 [options]
      
      [options]
      -h show this help 
      -u <uname>  : Enter the user name in remote server uname@domain
      -c          : Replace the meep_openpiton repo on server. Default is off.
      -j <num>    : number of parallel jobs passed to makefile. default is 16.
      -v <version>: Verilatot version number. Default is $VERILATOR_VERSION
      -m <bash commands> : bash command that should be run on server. e.g:\"module load gcc/11.3.0 python/3-intel-2021.3\"
      -p          : Initiate remote server for continit
      "      
      exit 0;

}

num=0;
check_point(){
  if [ $enable_checkpoints == 0 ]; then return;  fi
  num=$((num+1))
  echo "step $num. Press enter to continue: "  
  read -n1 kbd
}

#Download & build DTC. In vcase the pre-compiled did not work anymore
compile_and_install_DCT(){
    echo "Download & build DTC then copy on remote server"
    mkdir -p ~/scratch/`whoami` 
    cd ~/scratch/`whoami`/ 
    git clone git://git.kernel.org/pub/scm/utils/dtc/dtc.git
    cd dtc 
    make
    scp -r ~/scratch/`whoami`/dtc  $uname:scratch/$server_uname/dtc        
}


while getopts "h?u:cpj:v:m:" opt; do
  case "$opt" in
    h|\?)
      help
      exit 0
      ;;
    c) copy_meep_repo=1
      ;; 
    p) CONTINT=1
      ;; 
    u) uname=$OPTARG
      ;;   
    j) NUM_JOBS=$OPTARG
      ;;
    v) VERILATOR_VERSION=$OPTARG
      ;;
    m) command="$OPTARG;"
  esac
done
shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift

if [ -z $uname ] 
then
    echo "Error : the server name is not given; ";
    help
    exit 1;
fi 

Vname=${VERILATOR_VERSION//./_}

root="$(basename -- $PITON_ROOT)"
server_root="${uname}:$root";

ssh="ssh -q $uname"

# Attempt to SSH to the server
$ssh "exit"
d=$?
# Check the exit status of the ssh command
if [ $d -ne 0 ]; then
    echo "Error: Unable to connect to $uname. connection return exit code $d!"
    exit 1  # Exit with an error code
fi


# Run multiple commands using a single SSH connection and capture the output
read -r server_uname csh_server SERVER_HOME sbatch_server ccache_server help2man_server <<< $($ssh '
    uname_output=$(whoami)
    csh_output=$(which csh 2>/dev/null)
    home_output=$(realpath ./)  
    sbatch_output=$(which sbatch 2>/dev/null) 
    ccache_output=$(which ccache 2>/dev/null) 
    help2man_output=$(which help2man 2>/dev/null) 
    echo "$uname_output \"$csh_output\" $home_output \"$sbatch_output\"  \"$ccache_output\" \"$help2man_output\""
')

 #echo "$uname_output $csh_output $home_output $sbatch_output  $ccache_output help2man_output"
 
 
#RISCV=$HOME/scratch/`whoami`/riscv_install
#RISCV_REPO_LOCAL=$RISCV/riscv-gnu-toolchain
RISCV_REPO_SERVER=scratch/$server_uname/riscv_install/riscv-gnu-toolchain
RISCV_BIN_SERVER=scratch/$server_uname/riscv_install/bin/riscv64-unknown-elf-cpp
LOCALPERL_SERVER=scratch/$server_uname/localperl/bin/perl

VERILATOR_REPO_LOCAL=~/scratch/`whoami`/verilator_repo
VERILATOR_REPO_SERVER=scratch/$server_uname/verilator_repo
VERILATOR_BIN_SERVER=scratch/$server_uname/verilator_${Vname}/bin/verilator
CSH_SERVER=scratch/$server_uname/lib/tcsh/bin/csh
HELP2MAN_SERVER=scratch/$server_uname/lib/help2man/bin/help2man
SBATCH_SERVER=scratch/$server_uname/slurm
CCACHE_SERVER=scratch/$server_uname/lib/ccache/bin/ccache
DTC_SERVER=scratch/$server_uname/dtc
SCRATCH_SERVER=$SERVER_HOME/scratch/$server_uname
LOCAL_PERL=$SCRATCH_SERVER/localperl/bin/perl

perl_uname1=`ssh $uname uname`
perl_uname2=`ssh $uname uname -m`
perl_uname="${perl_uname1}-${perl_uname2}"



declare -A urls
urls=( ["perl"]="https://www.cpan.org/src/5.0/perl-5.34.0.tar.gz" \
          ["ccache"]="https://github.com/ccache/ccache/releases/download/v4.3/ccache-4.3.tar.gz" \
          ["tcsh"]="https://launchpad.net/ubuntu/+archive/primary/+sourcefiles/tcsh/6.21.00-1/tcsh_6.21.00.orig.tar.gz" \
          ["Bit-Vector"]="https://www.cpan.org/modules/by-module/Bit/Bit-Vector-7.4.tar.gz" \
          ["help2man"]="https://ftp.gnu.org/gnu/help2man/help2man-1.48.3.tar.xz" \
) 

declare -A tars                              
tars=( ["perl"]="tar -xzf perl-5.34.0.tar.gz" \
          ["ccache"]="tar -xzf ccache-4.3.tar.gz" \
          ["tcsh"]="tar -xzf tcsh_6.21.00.orig.tar.gz" \
          ["Bit-Vector"]="tar -xzf Bit-Vector-7.4.tar.gz" \
          ["help2man"]="tar -xvf help2man-1.48.3.tar.xz" \
) 

declare -A installs
installs=( ["perl"]="./Configure -des -Dprefix=$SCRATCH_SERVER/localperl; make; make install" \
           ["ccache"]="mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release .. -DZSTD_FROM_INTERNET=ON;make; mkdir -p $SCRATCH_SERVER/lib/ccache/bin/; cp $SCRATCH_SERVER/ccache-4.3/build/ccache  $CCACHE_SERVER;" \
           ["tcsh"]="mkdir -p $SCRATCH_SERVER/lib/tcsh; ./configure --prefix=$SCRATCH_SERVER/lib/tcsh; make; make install; cd $SCRATCH_SERVER/lib/tcsh/bin; ln -s tcsh csh " \
           ["Bit-Vector"]="$LOCAL_PERL Makefile.PL; make; make install; mkdir -p $root/piton/tools/perlmod/$perl_uname; cp -r $SCRATCH_SERVER/localperl/lib/site_perl/5.34.0/*/* $root/piton/tools/perlmod/$perl_uname" \
           ["help2man"]="mkdir -p $SCRATCH_SERVER/lib/help2man;./configure --prefix=$SCRATCH_SERVER/lib/help2man; make; make install; " \
)                              

install_package_locally(){
    pkg_name=$1
       
    # Check if the package name exists in the URLs associative array
    if [[ -z "${urls[$pkg_name]}" ]]; then
        echo "Package '$pkg_name' not found in the URLs array."
        return 1
    fi

    # Download the package
    echo "Downloading ${pkg_name} from ${urls[$pkg_name]}..."
    wget "${urls[$pkg_name]}"
    
    # Get the tar file name from the URL
    tar_name=$(basename "${urls[$pkg_name]}")
    
    # Copy the tar file to the remote server
    echo "Copying $tar_name to $uname:scratch/$server_uname/..."
    scp "$tar_name" "$uname:scratch/$server_uname/"
    
    # Execute commands on the remote server
    $ssh "cd scratch/$server_uname/ && \
                  ${tars[$pkg_name]} && \
                  cd ${tar_name%.tar.*} && \
                  ${installs[$pkg_name]}"    
    # Optional: Clean up local tar file after installation
    echo "Cleaning up..."
    rm "$tar_name"
}
    





check_point

if [ $csh_server = \"\" ]; then 
    add_csh_to_path=1
    if $ssh "! test -e $CSH_SERVER"; then
        echo "[info] csh is not installed in the remote server.It will be installed locally by this script" 
        install_csh=1               
    fi
fi


if [ $help2man_server = \"\" ]; then 
    add_help2man_to_path=1
    if $ssh "! test -e $HELP2MAN_SERVER"; then
        echo "[info] help2man is not installed in the remote server.It will be installed locally by this script" 
        install_help2man=1               
    fi
fi


if [ $copy_meep_repo == 0 ];then
  echo "[info] -c option was not passed to script. skip replacing meep_repo"
fi


if [ $CONTINT == 1 ];then
      echo "[info] -p option is passed to script. install continet library"
      #check if perl is installed locally on remote server
      add_perl_to_path=1 
      if $ssh "test -e $LOCALPERL_SERVER"; then  
        echo "[info] Local Perl $LOCALPERL_SERVER exists on remote server. Skip installing perl on the server"
        install_perl=0
      fi    
        
      if [ $ccache_server = \"\" ]; then 
#        add_ccache_to_path=1
        if $ssh "! test -e $CCACHE_SERVER"; then
            echo "[info] ccache is not installed in the remote server.It will be installed locally by this script" 
#            install_ccache=1               
        fi
      fi



      if [ $sbatch_server = \"\" ]; then 
        add_sbatch_to_path=1
        if $ssh "! test -e $SBATCH_SERVER"; then
            echo "[info] sbatch is not installed in the remote server. A fake slurm queu will be installed on the server. Note that you can set the number of jobs that can be run in parallel by setting MAX_RUN value in $SCRPT_DIR_PATH/src/fake_slurm/fake_sbatch.sh. Default is 2." 
            install_sbatch=1               
        fi
      fi     
fi   
  

#check if iscv-gnu-toolchain installed on remote server
if $ssh "test -e $RISCV_BIN_SERVER"; then
  echo "[info] RISCV toolchain is installed on remote server. Skip toolchain uploading/compilation"
  copy_riscv_repo=0
  install_rsicv_server=0

#check if iscv-gnu-toolchain exist in remote server
elif $ssh "test -e $RISCV_REPO_SERVER"; then   
    echo "[info] RISCV repo exists on remote server. Skip uploading it to the server"
    copy_riscv_repo=0
fi

#check if verilator installed on remote server
if $ssh "test -e $VERILATOR_BIN_SERVER"; then
  echo "[info] VERILATOR $VERILATOR_VERSION is installed on remote server. Skip verilator uploading/compilation"
  copy_verilator_repo=0
  install_verilator_server=0
fi

#check if DTC exist in remote server
if $ssh "test -e $DTC_SERVER"; then   
    echo "[info] DTC exists on remote server. Skip uploading it to the server"
    copy_dtc_server=0
fi




check_point




#copy openpiton repo to the server
if [ $copy_meep_repo == 1 ] 
then
    echo "[info] Copy $root to remote server"
    $ssh mkdir -p $root/build
#    scp -r $PITON_ROOT/piton $server_root/
    rsync -avz $EXCLUDES $PITON_ROOT/piton/* $server_root/piton
    $ssh mkdir -p scratch/$server_uname

  for str in ${mySetups[@]}; do
      if $ssh "! test -e  $root/$str"; then 
        echo "[info] $root/$str does not exsited. Skip updating this file!"
        continue
      fi
      echo "[info] update $str"
      $ssh "sed -i \"s|export[ ]*RISCV=|export RISCV=$SCRATCH_SERVER/riscv_install  #|g\" $root/$str" 
      $ssh "sed -i \"s|export[ ]*VERILATOR_ROOT=|export VERILATOR_ROOT=$SCRATCH_SERVER/verilator_${Vname}  #|g\" $root/$str"
      $ssh echo "export PATH=\\\$PATH:$SCRATCH_SERVER/dtc   >> $root/$str "    
      if [ $add_perl_to_path == 1 ] 
      then
      $ssh echo "export PATH=$SCRATCH_SERVER/localperl/bin:\\\$PATH   >> $root/$str "
      $ssh echo "export DRMJOBSCRATCHSPACE=\\\$PITON_ROOT/build/var     >> $root/$str "
      $ssh "sed -i \"s|export[ ]*PERL_CMD|export PERL_CMD=$LOCAL_PERL  #|g\" $root/piton/piton_settings.bash" 
      $ssh "sed -i \"s|setenv[ ]*PERL_CMD|setenv PERL_CMD $LOCAL_PERL  #|g\" $root/piton/piton_settings.cshrc"  
       #change unpermited path in pal
      $ssh "mkdir -p $root/build/pal_tmp"
      $ssh "sed -i \"s|/usr/tmp/|\\\$ENV{DV_ROOT}/../build/pal_tmp/|g\" $root/piton/tools/local/pal/1.13/bin/pal"          
      #support verilator in contint
      $ssh "sed -i \"s|icv|vlt|g\" $root/piton/tools/src/contint/contint,1.0 "     
      fi
      if [ $add_csh_to_path == 1 ]; then 
      $ssh echo "export PATH=$SCRATCH_SERVER/lib/tcsh/bin:\\\$PATH   >> $root/$str "
      $ssh "sed -i \"s|/bin/csh -f|/usr/bin/env csh|g\" $root/piton/tools/bin/bw_cpp" 
      fi
      if [ $add_sbatch_to_path == 1 ]; then 
      $ssh echo "export PATH=$SCRATCH_SERVER/slurm:\\\$PATH   >> $root/$str "
      fi
      if [ $add_ccache_to_path == 1 ]; then
      $ssh echo "export PATH=$SCRATCH_SERVER/lib/ccache/bin:\\\$PATH   >> $root/$str "
      fi
      if [ $add_help2man_to_path == 1 ]; then 
      $ssh echo "export PATH=$SCRATCH_SERVER/lib/help2man/bin:\\\$PATH   >> $root/$str "
      fi
  done
   
  

  echo "[info] remove trace_hart_00.dasm fwrite from ariane.sv"
  $ssh "sed -i \"s|\\\$fwrite(f,|//\\\$fwrite(f, |g\" $root/piton/design/chip/tile/ariane/core/cva6.sv"
  echo "[info] remove git version read command from riscvlib.py"
  $ssh "sed -i \"s|piton_ver[ ]*=|piton_ver = \\\"123456789ABCDEF\\\" #|g\" $root/piton/tools/bin/riscvlib.py" 
  $ssh "sed -i \"s|ariane_ver[ ]*=|ariane_ver = \\\"123456789ABCDEF\\\" #|g\" $root/piton/tools/bin/riscvlib.py" 
    
fi


 

if [ $install_perl == 1 ];then
   #download perl source
  
   install_package_locally 'perl'
   install_package_locally 'Bit-Vector'
 
  
   #wget https://www.cpan.org/src/5.0/perl-5.34.0.tar.gz
   #scp  perl-5.34.0.tar.gz $uname:scratch/$server_uname/
   #$ssh "cd scratch/$server_uname/; tar -xzf perl-5.34.0.tar.gz"
   #$ssh "cd scratch/$server_uname/perl-5.34.0; ./Configure -des -Dprefix=$SCRATCH_SERVER/localperl "
   #$ssh "cd scratch/$server_uname/perl-5.34.0; make "
   #$ssh "cd scratch/$server_uname/perl-5.34.0; make install "
   #scp -r $SCRPT_DIR_PATH/src/Bit-Vector-7.4.tar.gz  $uname:scratch/$server_uname/ 
   #$ssh "cd scratch/$server_uname/; tar -xzf Bit-Vector-7.4.tar.gz"
   #$ssh "cd scratch/$server_uname/Bit-Vector-7.4; $LOCAL_PERL Makefile.PL "
   #$ssh "cd scratch/$server_uname/Bit-Vector-7.4; make "
   #$ssh "cd scratch/$server_uname/Bit-Vector-7.4; make install "
   
   #copy Bit-Vector to OP 
   #$ssh "mkdir -p $root/piton/tools/perlmod/$perl_uname; cp -r scratch/$server_uname/localperl/lib/site_perl/5.34.0/*/* $root/piton/tools/perlmod/$perl_uname"
    

   #clean mpi dir
   #rm perl-5.34.0*
fi

if [ $install_ccache == 1 ];then
   install_package_locally 'ccache'
   # wget https://github.com/ccache/ccache/releases/download/v4.3/ccache-4.3.tar.gz
   # scp  ccache-4.3.tar.gz $uname:scratch/$server_uname/
   # $ssh "cd scratch/$server_uname/;  tar -xvf ccache-4.3.tar.gz "
   # $ssh "cd scratch/$server_uname/ccache-4.3; mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release .. -DZSTD_FROM_INTERNET=ON;make;"
   # $ssh "mkdir -p scratch/$server_uname/lib/ccache/bin/; cp scratch/$server_uname/ccache-4.3/build/ccache  $CCACHE_SERVER;"
   # rm ccache-4.3.tar.gz
fi

if [ $install_csh == 1 ]; then 
  install_package_locally 'tch'
  #  scp -r $SCRPT_DIR_PATH/src/tcsh_6.21.00.orig.tar.gz  $uname:scratch/$server_uname/tcsh.tar.gz
  #  $ssh "cd scratch/$server_uname/;  tar -xvf tcsh.tar.gz; \
  #  mkdir -p $SCRATCH_SERVER/lib/tcsh; \
  #  cd tcsh-6.21.00; ./configure --prefix=$SCRATCH_SERVER/lib/tcsh; make; make install; "
  #  $ssh "cd scratch/$server_uname/lib/tcsh/bin; ln -s tcsh csh "
fi

if [ $install_sbatch == 1 ]; then 
    scp -r $SCRPT_DIR_PATH/src/fake_slurm  $uname:scratch/$server_uname/slurm    
fi

if [ $install_help2man == 1 ]; then 
  install_package_locally 'help2man' 
fi



  #settup benchmarks
  #cd $SCRPT_DIR_PATH/benchmark/app-scalar;  bash settup.sh
  
  #$ssh "cd $root/piton/tools/metro_mpi/benchmark/app-scalar; bash settup.sh"
  

check_point
#Copy riscv toolchain installer to remote server
if [ $copy_riscv_repo == 1 ] ; then
  $ssh mkdir -p scratch/$server_uname/riscv_install/
  #scp -r  $RISCV_REPO_LOCAL $uname:scratch/$server_uname/riscv_install/
fi

check_point
#Compiling RISC-V Toolchain on remote server
if [ $install_rsicv_server == 1 ]; then  
  scp -r $SCRPT_DIR_PATH/src/riscv_install.tar.xz  $uname:scratch/$server_uname/
  check_point
  $ssh "cd scratch/$server_uname/;  tar -xvf riscv_install.tar.xz "
fi


#Download and compile  libmpfr.so.6 / libmpfr.so.4 libs
#echo  "Download and compile  libmpfr.so.6 / libmpfr.so.4 libs and copy in remote server riscv/lib folder";
#cp  $SCRPT_DIR_PATH/src/mpfr4_4.0.1.orig.tar.xz  ~/scratch/`whoami`/
#cd  ~/scratch/`whoami`/
#tar -xvf mpfr4_4.0.1.orig.tar.xz
#cd mpfr-4.0.1
#mkdir -p ~/scratch/`whoami`/tmp_lib
#./configure --prefix=$ENV{HOME}/scratch/`whoami`/tmp_lib
#make 
#make install
#scp ~/scratch/`whoami`/tmp_lib/lib/libmpfr.so.6 $uname:scratch/$server_uname/riscv_install/lib/libmpfr.so.6
#scp ~/scratch/`whoami`/tmp_lib/lib/libmpfr.so.6 $uname:scratch/$server_uname/riscv_install/lib/libmpfr.so.4

check_point

#Copy verilator installer to remote server
if [ $copy_verilator_repo == 1 ] ; then
  #check if verilator repo exist locally
  if  ! [ -d "$VERILATOR_REPO_LOCAL/verilator/configure" ]; then
    echo "[info] Download verilator repo on local system";              
  fi
  cd $PITON_ROOT
  cp $SCRPT_DIR_PATH/src/verilator_build.sh piton/verilator_build.sh
  bash  piton/verilator_build.sh -d -v $VERILATOR_VERSION
  if [ $? -ne 0 ]; then
    echo "Git checkout was unsuccessful!"
    exit 1;
    # Additional commands to execute if checkout was successful
  fi


  
  check_point
        
  echo "[info] Copy verilator repo to the remote server $server_uname"
  $ssh mkdir -p scratch/$server_uname/
  $ssh rm -rf  $VERILATOR_REPO_SERVER
  #scp -r  $VERILATOR_REPO_LOCAL $uname:$VERILATOR_REPO_SERVER
  rsync -av $VERILATOR_EXLUDE  $VERILATOR_REPO_LOCAL/* $uname:$VERILATOR_REPO_SERVER
  
fi
check_point

#Compiling VERILATOR Toolchain on remote server
if [ $install_verilator_server == 1 ]; then  
  echo "[info] Compiling Verilator on remote server"
  scp $SCRPT_DIR_PATH/src/verilator_build.sh $uname:$root/piton/
  $ssh "$command cd $root;  bash piton/verilator_build.sh -i -j 8 -v $VERILATOR_VERSION"    
fi
check_point

#copy DCT on remote server
if [ $copy_dtc_server == 1 ]; then  
  echo "[info] Copy dtc repo to the remote server $server_uname"
  scp -r $SCRPT_DIR_PATH/src/dtc_install.tar.xz  $uname:scratch/$server_uname/
  check_point
  $ssh "cd scratch/$server_uname/;  tar -xvf dtc_install.tar.xz "
fi
check_point




    
echo "--------------------------------------------"
echo "    Initialization of server is completed!"
echo "--------------------------------------------"


exit 0;

