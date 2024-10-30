#!/usr/bin/perl

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


use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/lib";
#use Capture::Tiny ':all';
use IO::CaptureOutput qw(capture qxx qxy);

use File::Basename;
use File::Path qw(make_path remove_tree);
use Getopt::Long;
use base 'Class::Accessor::Fast';
use Cwd;
use Cwd 'abs_path';
use Getopt::Std;
#use GD::Graph::bars;
use List::MoreUtils qw(uniq);
use Chart::Gnuplot;
use List::Util qw(max sum min);


my $lat_perfix="Lat_";
my $uart_perfix="Uart_";
#use Archive::Zip qw( :ERROR_CODES :CONSTANTS );
#use Archive::Zip::MemberRead;

my $piton_root = $ENV{PITON_ROOT};

my $piton_is_set=1;
if (!defined $piton_root){
    $piton_root= abs_path(__FILE__."/../../../../..");    
    $piton_is_set=0;
    print "Warning environment variable PITON_ROOT is not defined. It is set to the following path by this script:\n\t $piton_root \n";    
}

my $mpi_sim_dir = abs_path(__FILE__."/../..");

#my $current_dir = getcwd;
my $sim_results_dir = "$piton_root/build/mpi_sims/sim_results";
my $model_bin_dir = "$piton_root/build/mpi_sims/models_bin";
my $chart_dir = "$piton_root/build/mpi_sims/chart-results";
my $tmp1 = "$piton_root/build/tmp1.sh";
my $tmp2 = "$piton_root/build/tmp2.sh";


my $repo = abs_path("$piton_root");
my $root= basename($repo);
my ($server_ref,$verilator_v,$sbatch) =get_server_id();
die "Error: SERVER setting is not defined in server.setting file! " if(!defined $server_ref);
my %server = %{$server_ref};
my $uname =$server{'UNAME'};
die "Error: uasername is not defined for the remote server! " if(!defined $uname);
my $ssh = "ssh -q $uname";
my $server_root = "${uname}:$root";
my $scp="scp";
my $run_on_server=0;
my $qta="\"";

#$current_dir="$current_dir/..";
my $ii=0;
my $server_models;
my $uart_results;

sub mean {
    return sum(@_)/@_;
}

sub stdev{
        my @data = @_;
        my $length = scalar @data;
        return 0 if($length == 1 || $length ==0);
        my $average = mean(@data);
        my $sqtotal = 0;
        foreach(@data) {
                $sqtotal += ($average-$_) *($average-$_);
        }        
        my $std = ($sqtotal / ($length-1)) ** 0.5;    
        return $std;
}

sub RSD {#relative standard deviation (RSD) Coefficient of variation
	 my $mean = mean(@_);
	 return 0 if($mean==0);
	 return (stdev(@_)*100)/mean(@_);
}

my %apps_hca = (
'mt-axpy'  => '~/risc-v-benchmarks/hpc_benchmarks/axpy',
'mt-matmul'=> '~/risc-v-benchmarks/hpc_benchmarks/gemm',
'mt-spmv'  => '~/risc-v-benchmarks/hpc_benchmarks/spmv',
'mt-somier'=> '~/risc-v-benchmarks/hpc_benchmarks/somier',
);

    
sub get_uart_results {
    my $result_dir =$sim_results_dir;
    make_path ($result_dir) unless(-d $result_dir );

    my $bash="cd $result_dir
    grep -R \"barrier\" -m 1 || true;";
    $uart_results= run_cmd_message_dialog_errors($bash);
   
    
    
    
}

sub check_we_have_output {
    my $bash=shift;    
    my($stdout,$exit,$stderr)=run_cmd_in_back_ground_get_stdout($bash);    
    my $miss = (length $stdout <2);
    return $miss;

}

sub copy_repo_on_server{
    my $bash = "cd $mpi_sim_dir; bash ./server_init.sh -u $uname -c";
    $bash.=  " -v $verilator_v" if defined ($verilator_v);
    system($bash);
}    

sub run_bash_cmd_get_stdout {
    my $cmd=shift;    
    $cmd= "$cmd 2>&1"; #redirects the standard error
    # Run the command and capture the output
    my  $stdout = `$cmd`;
    # Check if the command was successful
    if ($? != 0) {
        print "Error: $cmd failed:  $stdout\n";
        return "";
    }
    return   $stdout;
}

    
sub get_list_of_all_models_dir_in_server {    
    my $bash = ($run_on_server)?
    "$ssh find -type f \\( -name \"Vmetro_tile\" -o -name \"Vmetro_fake_mem\" -o -name \"Vmetro_chipset\" -o -name \"flist\" \\) -path \"./$root/build/METRO_MPI_ALL/*\" " 
    
    : "$ssh find -type f \\\\\\\( -name \"Vmetro_tile\" -o -name \"Vmetro_fake_mem\" -o -name \"Vmetro_chipset\" -o -name \"flist\" \\\\\\\) -path \\\"./$root/build/METRO_MPI_ALL/*\\\" " ;
    #print "$bash\n";
    my $out = run_cmd_message_dialog_errors($bash);
    $server_models =   "$out";   
}    
                
sub add_active_job{
    my ($id)=@_;
}

sub get_server_id {
    my $paths_file= "$mpi_sim_dir/server.setting";
    my $obj;
    system("cp $mpi_sim_dir/servers/mn5 $paths_file") unless (-f     $paths_file );
    
    if (-f     $paths_file ){
        $obj= do $paths_file;
    }else{    
        print "Error: Cannot load $paths_file file\n";
        exit 1;        
    }
    
    my $server_ref=$obj->object_get_attribute('SERVER');
       
    my $sbatch;
    my $sbatch_ref = $obj->object_get_attribute('SBATCH');    
    if(defined $sbatch_ref){
	    my %hash = %{$sbatch_ref};
	    $sbatch="";
	    foreach my $p (sort  {$a <=> $b} (keys %hash)){
	      $sbatch.="$hash{$p} \n"
	    }	
    }
 
    my $verilator_v =  $obj->object_get_attribute('VERILATOR_VERSION');   
    return ($server_ref,$verilator_v,$sbatch);
}

my $start_old = time();

sub my_run{
	my ($cmd,$timeout,$verbus)=@_;   
    $timeout= 10 if(!defined $timeout);
    $verbus = 0 if(!defined $verbus);
    $cmd =~ s/;\s*$/ /; #remove last ';' in command if exsited.
    my $bash= " bash -c \' $cmd; sleep $timeout; wait; \'" ;
    print " $cmd \n" unless ($verbus == 0);
    my $out = run_cmd_message_dialog_errors($bash);
    print "$out\n" unless ($verbus == 0);
    my $current_time = time();
    my $time =$current_time-$start_old;
    $start_old=$current_time;
    my $timestamp = localtime(time);
    print "\n  $timestamp : time taken from last checkpoint  $time Second\n"  unless ($verbus == 0);
}


sub run_cmd_in_back_ground
{
  my $command = shift;
  #print "\t$command\n";
  ### Start running the Background Job:
    my $proc = Proc::Background->new($command);
    my $PID = $proc->pid;
    my $start_time = $proc->start_time;
    my $alive = $proc->alive;

  ### While $alive is NOT '0', then keep checking till it is...
  #  *When $alive is '0', it has finished executing.
  while($alive ne 0)
  {
    $alive = $proc->alive;
    # This while loop will cause Gtk3 to continue processing events, if
    # there are events pending... *which there are...
    usleep(1000);
  }

    my $end_time = $proc->end_time;
    # print "*Command Completed at $end_time, with PID = $PID\n\n";
      # Since the while loop has exited, the BG job has finished running:
    # so close the pop-up window...
     # $popup_window->hide;

    # Get the RETCODE from the Background Job using the 'wait' method
    my $retcode = $proc->wait;
    $retcode /= 256;


    ### Check if the RETCODE returned with an Error:
    if ($retcode ne 0) {
        print "Error: The Background Job ($command) returned with an Error...!\n";
        return 1;
    } else {
        #print "Success: The Background Job Completed Successfully...!\n";
        return 0;
    }
    
}


sub run_cmd_in_back_ground_get_stdout
{
    my $cmd=shift;
    #open(OLDERR, ">&STDERR");
    #open(STDERR, ">>/tmp/tmp.spderr") or die "Can't dup stdout";
    #select(STDOUT); $| = 1;     # make unbuffered
    #print OLDERR "";  #this fixed an error about OLDERR not being used
    ## do my stuff here.
    
    STDOUT->flush();
    STDERR->flush();
    

     #   my ($stdout, $stderr, $exit) = capture {
   #       system( $cmd);
  my  ($stdout, $stderr,$success, $exit) = qxx( $cmd );
  #      };

#       print $stdout;
#    capture { $exit=run_cmd_in_back_ground($cmd) } \$stdout, \$stderr;
    
    #close(STDERR);
    #open(STDERR, ">&OLDERR");
    return ($stdout,$exit,$stderr);
    
}        
    
sub run_cmd_message_dialog_errors{
    my ($cmd)=@_;
    my ($stdout,$exit,$stderr)=run_cmd_in_back_ground_get_stdout($cmd);
    if(length $stderr>1){            
        print "Error : $cmd failed: $stderr\n";
        exit 1;
        #return 1;
    }if($exit){
        print "Error : $cmd failed: $stdout\n";
        #return 1;        
    }
    $stdout = "" if (!defined $stdout);
    return     $stdout
    
}

sub my_server_run {
    my ($cmd,$w,$core_num,$mode,$verbus,$run_time_hour)=@_;
    $mode = 'run' if (!defined $mode);
    $verbus = 0 if(!defined $verbus);
    my $queue = ($mode eq 'run')? $server{'RUN_QUEUE'}  : $server{'BUILD_QUEUE'} ;
    my $alloc_time = ($mode eq 'build') ?   "$server{'BUILD_TIME'}" :  "$server{'RUN_TIME'}";
    my $core_per_node=$server{'MAX_CORE_PER_NODE'}; 
    my $mpi_alloc_mode=$server{'MPI_JOB_ALLOCATION_MODE'};
    
    my $sbatch_task_config =
       ($mpi_alloc_mode eq '1' ) ? 
"#SBATCH --cpus-per-task=$core_num
#SBATCH --ntasks=1
"  :
"
#SBATCH --cpus-per-task=1
#SBATCH --ntasks=$core_num
";


    my $file="#!/bin/bash
cd  ~/$root/piton/tools/metro_mpi
$cmd
";
    my $job_id;
    $job_id=0;
    save_file("$tmp1",$file);
    my $work="~/meep_openpiton/$w/script";
    my $job=$w;
    $job =~ s/\//-/g;
    $job.=$job_id;
    $job_id++;


    my $node_num = int ($core_num/int($core_per_node));
    $node_num++;

# SLURM job script file.
# Modify this script as needed based on the specific SLURM configuration and resources available on your server.    
    $file="#!/bin/bash
#SBATCH --job-name=\"$job\"
#SBATCH --output=r.out
#SBATCH --error=r.err
#SBATCH --nodes=$node_num
#SBATCH --qos=$queue
#SBATCH --time=$alloc_time

$sbatch_task_config
$sbatch

mkdir -p ./out;
time bash ./build.sh > ./out/${job}_log

";

   save_file("$tmp2",$file); 
   
    
    #create run.sh file    
    #my $bash="$ssh mkdir -p $root/$w/script; $scp $tmp1 $server_root/$w/script/build.sh; $scp $tmp2 $server_root/$w/script/Queue_build.sh; $ssh chmod +x $root/$w/script/Queue_build.sh";
    
    # my $bash="sftp $uname << EOF  
    # mkdir -p $root/$w/script; 
    # put $tmp1 $server_root/$w/script/build.sh; 
    # put $tmp2 $server_root/$w/script/Queue_build.sh;    
    # chmod 755 $root/$w/script/Queue_build.sh
    # EOF";
   
    my $bash="$scp $piton_root/build/tmp*.sh  $server_root/$w/; 
    $ssh $qta mkdir -p $root/$w/script; 
    cp $root/$w/tmp1.sh  $root/$w/script/build.sh;   
    cp $root/$w/tmp2.sh  $root/$w/script/Queue_build.sh;  
   
    chmod +x $root/$w/script/Queue_build.sh;
    cd $root/$w/script/; sbatch Queue_build.sh $qta";

    
    my_run($bash,0,$verbus);
    
    #$bash="$ssh \" cd $root/$w/script/; sbatch Queue_build.sh\"";
    #my $id = run_cmd_message_dialog_errors($bash);    
    #$id=~ s/[^0-9]//g;
    #add_active_job($id);

    #wait_until_squeue_is_empty() if ($wait);


    return;
}


sub my_server_run_sequencial {
    my ($cmd,$verbus)=@_;
    my $file="#!/bin/bash
$cmd
";
    save_file("$tmp1",$file);
    my $bash="$scp $tmp1 $server_root/bash.sh;   $ssh chmod +x $root/bash.sh; $ssh $root/bash.sh";
    my_run($bash,0,$verbus);
    return;
}


sub get_process_time{
	my $start=shift;
	my $current_time = time;
	my $seconds = $current_time - $start;
	my $days = int($seconds / (24 * 60 * 60));
	my $hours = int(($seconds % (24 * 60 * 60)) / (60 * 60));
	my $minutes = int(($seconds % (60 * 60)) / 60);
	my $sec = $seconds % 60;
	my $time = "";
	$time.= "${days}D: " if($days>0) ;
	$time.= "${hours}H: "    if($hours>0);
	$time.= "${minutes}M: " if($minutes>0);
	$time.= "${sec}S";
	return $time;
}

sub adjast_len{
	my ($ref,$name)=@_;
	
	for (my $t=length($name); $t<length($ref) ; $t++){
		$name= ($t & 1)? " ".$name : $name." ";
	} 
	return $name;
}

sub wait_until_squeue_is_empty{
    my @list;
    my $timestamp = localtime(time);
    my $start =time;    
    my $last_info="";
    my @hdrs=("      Time      ","  Total " , " Active ", " Ended ", "   Status    " );    
    print "\nWait for all jobs in slurm queue to finish\n";
    print "\n           Start at   $timestamp\n";
    foreach my $p (@hdrs){
        print "_" x int(length($p)+1);
    }
    print "\n";
    foreach my $p (@hdrs){
        print "$p|";
    }
    print "\n";
    my $max_job=0;
    my $len=0;
    my $loop=0;
    do{
       
        my $bash="$ssh squeue -u `$ssh whoami`";
        my $id = run_cmd_message_dialog_errors($bash);            
                
        @list = split("\n",$id);
        my @names;
        my $num=0;
        my %sq;
        my %st;

        foreach my $line (@list){
            if($num==0){
               @names = split /\s+/, $line ;
            } else{
               my @values = split /\s+/, $line ;
               foreach my $f (@names){
                  my $v= remove_all_white_spaces(shift(@values));
                  $f=remove_all_white_spaces($f);
                  $sq{$num-1}{$f}=$v;
                  if($f eq "ST") {
                    $st{$v} = ( defined $st{$v})? $st{$v}+1 : 1;
                  }
               }
            }
            $num++;
        }
        #print $id;
        my $t=$num-1;
        $max_job = $t if($max_job<$t);
        my $time = get_process_time($start);       
        my $done = $max_job-$t;
        my $info="";
       
        foreach my $p (sort keys %st) {
            $info.= "$p: $st{$p}  "
        }        
        my @body=($time,$max_job,$t,$done,$info);
        my $i=0;        
        my $txt ="";
        foreach my $p (@body){
                $txt .= adjast_len($hdrs[$i],$p)."|";
                $i++;
        }
        

        if( $last_info eq $info ) {
            #remove last line
            print "" x $len . "\r";
        }else {
            print "\n" if($len);
        }
        $last_info=$info;
        print ($txt);        
        $len = length($txt);
        #print "len = $len";
        my $sleep = ($loop<10)? 10 : 60;
        sleep $sleep;
        $loop++;
    
    }while(defined $list[1]);
    
    print "\n";
    
    foreach my $p (@hdrs){
        print "_" x int(length($p)+1);
    }
       
    $timestamp = localtime(time);
    print "\n           Ended at   $timestamp\n\n";    
    
}

sub get_pronoc_xy {
    my ($x_mul,$y_mul,$piton_x,$piton_y) =@_;
    my $pronoc_x = int($piton_x * $x_mul);
    my $pronoc_y = int($piton_y * $y_mul);
    return ($pronoc_x , $pronoc_y);
}



sub  gen_model_name {
    my ($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat)=@_;
    my $core_name=$core->{name};
    my $cache_name = get_cache_conf_name($core);
    my $mode_name;
    my $size= ${mesh_dim} * ${mesh_dim};
    my $n1 ="${core_name}_${cache_name}_M${size}_";
    my $n2 =    ($noc->{noc} eq'pronoc')? "p_" : "o_";
    $n2.= "v".$noc->{V};
    $n2.= ($noc->{SSA} eq "YES") ? "_A1_" : "_A2_";
    $n2.= "S".$noc->{SMART}."_";
    $n2.= "R".$noc->{PIPE_REG};
    if($noc->{CONCENT} > 1 ){
        my  ($pronoc_x , $pronoc_y) = get_pronoc_xy($noc->{X_MUL},$noc->{Y_MUL},$mesh_dim,$mesh_dim);
        $n2.= "x".$pronoc_x; 
        $n2.= "y".$pronoc_y;
        $n2.= "z".$noc->{CONCENT};
    }

    $n2.= ($noc->{ARBITER} eq "RRA" ) ?  "_W0_" : "_W1_";
    $n2.=$noc->{ROUTE};
    
    my $nw="";
    for (my $i=1;$i<=3;$i++){
    	next if(!defined $noc->{"noc${i}_width"});
    	my $l=$noc->{"noc${i}_width"};
   		$nw.="_$l";    
    }
    $n2.="nw$nw" if(length($nw)>1);
    
   
    my $n4 = ($mc_loc eq "all_edges") ? "E4" :
    ($mc_loc eq "both_y_edges") ? "E2" :
    ($mc_loc eq "first_y_edge") ? "E1" : 
    $mc_loc;#custom
    
    my $n3 = ($n4 eq $mc_loc)? "" : "_mc$mc";
    
    my $mo="";
	if(defined $mc_lat->{delay}) {
		my @ll = split('/', $mc_lat->{delay});	$mo=$ll[-1];
	}
	my $n5 = 
		($mc_lat->{enable} eq "No") ? "" :
		($mc_lat->{delay}   =~ /^\d+$/) ? 'L'.$mc_lat->{delay} :
		$mo;		

    $mode_name="$n1$n2$n3$n4$n5";
    return $mode_name;
}


sub get_edge_routers_ids{
    my ($x, $y, $mc_loc)=@_;
    my @edge_ids =
    ($mc_loc eq "all_edges" ) ? get_both_xy_edge_routers_ids ($x,$y) :
    ($mc_loc eq "both_y_edges" ) ? get_both_y_edge_routers_ids ($x,$y) :
    ($mc_loc eq "first_y_edge") ? get_first_y_edge_routers_ids($x,$y) :
    get_both_xy_edge_routers_ids ($x,$y) ;#get ll edges for custom mmc map
    return @edge_ids;
}



sub get_both_xy_edge_routers_ids{
    my ($x, $y)=@_;
    my @edges;
    my $i;
    my $j;
    
    $j=0;
    for ($i=0; $i<$x; $i++){my $id= $j*$x+$i;push (@edges,$id);} # first row
    
    $i=$x-1;
    for ($j=1; $j<$y; $j++){my $id= $j*$x+$i;push (@edges,$id);} #last column
    
    $j=$y-1;
    for ($i=$x-2; $i>0; $i--){my $id= $j*$x+$i;push (@edges,$id);} # last row
    
    $i=0;
    for ($j=$y-1; $j>0; $j--){my $id= $j*$x+$i;push (@edges,$id);} #first column
    
    #print join(',',@edges), "\n";
    return uniq(@edges);
}

sub get_both_y_edge_routers_ids{
    my ($x, $y)=@_;
    my @edges;
    my $i;
    my $j;

    $i=0;
    for ($j=$y-1; $j>=0; $j--){my $id= $j*$x+$i;push (@edges,$id);} #first column
    
    $i=$x-1;
    for ($j=0; $j<$y; $j++){my $id= $j*$x+$i;push (@edges,$id);} #last column
    
    #print join(',',@edges), "\n";
    return uniq(@edges);
}


sub get_first_y_edge_routers_ids{
    my ($x, $y)=@_;
    my @edges;
    my $i;
    my $j;
    $i=0;
    for ($j=$y-1; $j>=0; $j--){my $id= $j*$x+$i;push (@edges,$id);} #first column
    return uniq(@edges);
}

sub distribute_mc_equal_space_on_all_edges {
    my ($x,$y,$mc,$mc_loc)=@_;   
    my @edge_ids =get_edge_routers_ids ($x,$y,$mc_loc);
    my $edges = scalar @edge_ids;
    my $d= ($mc==0)? 0 : ($edges/$mc);    
    my $mc_num=0;
    my $mmc =  '';    
    for (my $m=0; $m<$mc; $m++){
        $mc_num= int ($d * $m);
        my $mc_id = $edge_ids[$mc_num];
        $mmc.=($m==0)?  "-m \"$mc_id" : ",$mc_id";         
    }
    $mmc.="\"" unless($mc==0);
  #  print "all_edges : @edge_ids  ; mmcs: $mmc \n";
    return $mmc
}

sub get_pronoc_size_with_concentration {
	my ($piton_nx , $piton_ny , $concent, $dim_max_diff)=@_;
	my $min=$piton_nx * $piton_ny * $concent * 2;
	my ($pronoc_x, $pronoc_y);
	for (my $x = 1; $x<=$piton_nx; $x++){
		for (my $y = 1; $y<=$piton_ny; $y++){
			my $size= $x * $y * $concent;
			my $diff = ($x<$y) ? $y - $x : $x-$y;
			if ( $size >= ($piton_nx * $piton_ny) && $diff<=$dim_max_diff && $size < $min ) {
				$min = $size; 
				$pronoc_x = $x;
				$pronoc_y = $y;				
			}			
		}
	} 
    return ($pronoc_x ,		$pronoc_y);
}

sub get_metro_mpi_command{
    my ($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat,$model_dir,$mode,$app_arg)=@_;
    my $x=${mesh_dim};
    my $y=${mesh_dim};
    my $p=' ';
    my $pronoc_params=" ";
    my $T1 = $x;
    my $T2 = $y;
    if($noc->{noc} eq'pronoc'){
        $p= '-p';
        $pronoc_params ="\"SMART_MAX=$noc->{SMART},ADD_PIPREG_AFTER_CROSSBAR=$noc->{PIPE_REG},SWA_ARBITER_TYPE=$noc->{ARBITER},WEIGHTw=$noc->{W}";
        if($noc->{CONCENT} > 1 ){
            ($T1 , $T2) = get_pronoc_xy($noc->{X_MUL},$noc->{Y_MUL},$mesh_dim,$mesh_dim);
            my $T3= $noc->{CONCENT};
            $pronoc_params.= ",T1=$T1,T2=$T2,T3=$T3";
        }    
        $pronoc_params.="\"";
    }
    
    
    
    my $core_name=$core->{name};

    #map memory controllers
    my $mmc = (
        ($mc_loc eq "all_edges") || 
        ($mc_loc eq "both_y_edges") || 
        ($mc_loc eq "first_y_edge" )) ? 
        distribute_mc_equal_space_on_all_edges($T1,$T2,$mc,$mc_loc):
        "-m \"$mc\"";
        
    my $flag= ($mode eq 'run') ? "-r" : ($mode eq 'build') ? '-c -t -f' : '-c -f -t -r';
    
    $app_arg= (defined $app_arg) ? "-a $app_arg" : " ";
    my $lat = ($mc_lat->{enable} eq "Yes")? "-l ".$mc_lat->{delay}  : "";
    my $cache= get_cache_conf($core);
    $cache .= " -dryrun " if ($mode eq 'build');     
    
    for (my $i=1;$i<=3;$i++){
    	next if(!defined $noc->{"noc${i}_width"});
    	my $l=$noc->{"noc${i}_width"};
   		$cache.=" -noc${i}_width=$l "    
    }
    my $comand = "";
    $comand.= "bash ./metro_mpi_run.sh -x $x -y $y -z $core_name -w $model_dir $p $pronoc_params $flag $app_arg $mmc $lat -d -e \"$cache\"";
    return $comand;
}

sub extract_exec_time_results{
    my ($result_file)=@_;
    my $bash="grep -R \"barrier\"  $result_file";
    my $r= run_cmd_message_dialog_errors($bash);
    if(!defined $r){
    	print "Error in $result_file: The text \"barrier\" is missing\n";
    }
    $r = capture_number_after (":",remove_all_white_spaces($r));
    return int($r)/1000;
}

sub clear_diags_path{
	my $bash = "$ssh  rm -rf $root/build/diags;";	
	my_run($bash,0,0); 
}

sub gen_model {
    my ($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat,$force,$verbus)=@_;
    my $mode_name =gen_model_name($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat);
    my $model_dir= "build/METRO_MPI_ALL/$mode_name";
    my $local_model_dir ="$model_bin_dir/${mode_name}";
    my @bin_mpi = (    "metro_chipset/rel-0.1/obj_dir/Vmetro_chipset",
        "metro_chipset/rel-0.1/flist",
        "metro_fake_mem/rel-0.1/obj_dir/Vmetro_fake_mem",
        "metro_fake_mem/rel-0.1/flist",
        "metro_tile/rel-0.1/obj_dir/Vmetro_tile",
        "metro_tile/rel-0.1/flist",    
    );
    
    my @paths =(
        "metro_chipset/rel-0.1/obj_dir/",
        "metro_fake_mem/rel-0.1/obj_dir/",
        "metro_tile/rel-0.1/obj_dir/",            
    );
    
    my $cmd;
    #check if the model existed locally
    my ($exsited_locally,$exsited_remotely) = check_model_bin($mode_name);

    if($force){
        my $bash=""; 
        if ($exsited_locally){
            print "Remove $root/$model_dir from local\n" if ($verbus==1);
            my $bash .= "rm -rf $local_model_dir;";
        }    
        if($exsited_remotely){
            print "Remove $root/$model_dir from the server\n" if ($verbus==1);
            $bash .= "$ssh  rm -rf $root/$model_dir;";
        
        }    
        if(length($bash)>1){
            my_run($bash,0,$verbus);
            get_list_of_all_models_dir_in_server();
        }
        $exsited_locally=0;
        $exsited_remotely=0;                            
    }
            
    if($exsited_locally==0 && $exsited_remotely==0){
        print "$mode_name does not exist in both local & remote server. Start Generateing $mode_name...\n";
        
        $cmd = get_metro_mpi_command($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat,$model_dir,'build');
        
        #build the model files with dryrun sequencually


        my $bash= " cd $root/piton/tools/metro_mpi;  $cmd; wait";
        print "Command run sequencial on server:\n $cmd\n" if ($verbus==1);
        my_server_run_sequencial($bash,$verbus);
       
        
       $cmd.=" -b; "; #run verilator build on locally copied files 
       
       #clean build dir 
       $cmd.="wait; cd ../../..; find ./$model_dir/*/rel-0.1/obj_dir/* -type f  ! -path \"*/Vmetro_tile\" ! -path \"*/Vmetro_fake_mem\" ! -path \"*/Vmetro_chipset\" ! -path \"*/flist\" -delete;  ";
	$cmd.="find ./$model_dir/*/rel-0.1/src_hdl/*  -delete";
       
        print "Command send to slurm queue:\n $cmd\n" if ($verbus==1);
        #generate model
        my_server_run($cmd,$model_dir,10,'build',$verbus);    
        
    }

    elsif($exsited_locally==0){
        #copy from remote server
        print "$mode_name does not exist in local but it exists in remote server. Copying  $mode_name to the local drive...\n";
        $cmd=" ";
        foreach my $ff (@bin_mpi){
            my $dd = dirname("$local_model_dir/$ff");
            $cmd.="mkdir -p $dd; ";
            $cmd.="$scp  $server_root/$model_dir/$ff $local_model_dir/$ff; ";    
        }
        
        my_run($cmd,0,$verbus);
        #foreach my $ff (@bin_mpi){
        #    my $local_name = "$local_model_dir/$ff";
        #    my $zip_name   = "${mode_name}/$ff";    
        #    my $member1 = $binzip->removeMember( "$zip_name" );
        #    my $file_member = $binzip->addFile( "$local_name", "$zip_name" );

        #}

        # Save the Zip file
        #unless ( $binzip->writeToFileNamed("$current_dir/models.zip") == AZ_OK ) {
        #    die 'Zip archive write error';
        #}
        
        #delet temp files
        #$cmd="rm -rf $local_model_dir";
        #my_run($cmd,1);    

    }
    elsif($exsited_remotely==0){
        #copy to remote server
        print "$mode_name does not exist in remote server but it exists in local drive. Copying $mode_name to the remote server ...\n";
        #$cmd =" rm -rf $model_bin_dir/${mode_name}; ";
        #$cmd .= "unzip \"$current_dir/models.zip\" \"${mode_name}/*\" -d \"$model_bin_dir\";";

        foreach my $ff (@paths){
            $cmd.="$ssh mkdir -p $root/$model_dir/$ff; ";        
        }
                
        foreach my $ff (@bin_mpi){
            $cmd.="$scp  $local_model_dir/$ff  $server_root/$model_dir/$ff;";    
        }
        #delet temp files
        #$cmd.="rm -rf $local_model_dir;";
        
        my_run($cmd,0,$verbus);            

    }else{
       
        print "$mode_name is generated before and exsits in both local and remore server! skip building the model\n";
    }    
}

sub copy_compilation_log_file{
    my (${mode_name},$verbus)=@_;
    my $local_model_dir ="$model_bin_dir/${mode_name}";
    my $model_dir= "build/METRO_MPI_ALL/$mode_name";
    my $cmd=" ";
    my $dd = dirname("$local_model_dir/logs");
    $cmd.="mkdir -p $dd; ";
    $cmd.= "$scp -r $server_root/$model_dir/{sims.log,/script/*}   $dd/";
    my_run($cmd,0,$verbus);   
}

sub problem_cal {
    my ($problem,$mesh_dim, $app)=@_;
    my $problem_dim = $app->{problem_dim};
    my $problem_base = $app->{problem_base};
    $problem_base=16 if (!defined $problem_base);
    my $scale = $app->{scale};
    #for weak scale the given problem is per core while in strong scale is for all cores
    my $c = ($scale eq 'W' ) ? $mesh_dim * $mesh_dim : 1;
    my $problem_total =     int (($problem_base *$c* $problem) ** (1/$problem_dim));    
    return $problem_total;
}

sub app_inst_name_gen {
    my ($problem,$mesh_dim, $app, $app_loc)=@_;
    my $problem_total =problem_cal($problem,$mesh_dim, $app);
    my $name= $app->{name};
    my $dtype= $app->{Dtype};    
    $dtype = "double" if (!defined $dtype);
    my $app_name = ($app_loc eq 'Local') ? "${name}_${problem_total}_${dtype}" : "hca-${name}_${problem_total}";
    if (${name} eq 'mt-stream'){
		my $pause = $app->{pause};
		my $ratio = $app->{ratio};
		$app_name= "${app_name}_${pause}_${ratio}";
	} 
	my $stk = $app->{stk};
	if ($stk != 128){ #add stock pointer sie if its not default size
	    $app_name = "${app_name}_S${stk}";
	}    
    return $app_name;
}

sub check_model_bin {
    my ($mode_name)=@_;
    #check if the model existed locally
    my $local_model_dir ="$model_bin_dir/${mode_name}";
    my $model_dir= "build/METRO_MPI_ALL/$mode_name";
    my $exsited_locally  =1;
    my $exsited_remotely =1;
    my @bin_mpi = (    "metro_chipset/rel-0.1/obj_dir/Vmetro_chipset",
        "metro_chipset/rel-0.1/flist",
        "metro_fake_mem/rel-0.1/obj_dir/Vmetro_fake_mem",
        "metro_fake_mem/rel-0.1/flist",
        "metro_tile/rel-0.1/obj_dir/Vmetro_tile",
        "metro_tile/rel-0.1/flist",    
    );
    #my $binzip = Archive::Zip->new();
    #unless ( $binzip->read( "$current_dir/models.zip" ) == AZ_OK ) {
    #    die 'Zip archive read error';
    #}

    foreach my $ff (@bin_mpi){
        #my $member2 = $binzip->memberNamed( "${mode_name}/$ff" );
        $exsited_locally= 0  unless(-e "$local_model_dir/$ff");
        #$exsited_locally= 0  unless (defined $member2);
        $exsited_remotely=0  unless( $server_models =~ m/$root\/$model_dir\/$ff/);        
    }

    return ($exsited_locally,$exsited_remotely);
}


sub check_result_valid_localy {
	my ($dir)=@_;
	#check results are valid locally
	my $fake_uart= "$sim_results_dir/$dir/fake_uart.log";
	my $bash="grep -R \"barrier\" -m 1 $fake_uart || true;";
    my $r= (-f "$fake_uart") ?  run_cmd_message_dialog_errors($bash)  : "";
    my $valid= (length($r) >3 ) ? 1 :0; 
	#check results are valid remotely 
	return $valid;
}

sub check_result_valid_remotely {
	my ($dir)=@_;
	my $fake_uart = "./$root/build/METRO_MPI_ALL/$dir/fake_uart.log";
	my $bash="$ssh $qta  [[ -f $fake_uart ]] && grep -R \"barrier\" -m 1 $fake_uart || echo 0$qta";
    my $r=run_cmd_message_dialog_errors($bash);
	my $valid= (length($r) >3 ) ? 1 :0; 
	return $valid;	
}



sub get_cache_conf {
    my $core=shift;
    my $cache="";
    $cache.= "-config_l1i_size=$core->{l1i_SZ}  ";
    $cache.= "-config_l1i_associativity=$core->{l1i_AC} ";
    $cache.= "-config_l1d_size=$core->{l1d_SZ} ";
    $cache.= "-config_l1d_associativity=$core->{l1d_AC} ";
    $cache.= "-config_l15_size=$core->{l15_SZ} ";
    $cache.= "-config_l15_associativity=$core->{l15_AC} ";
    $cache.= "-config_l2_size=$core->{l2_SZ} ";
    $cache.= "-config_l2_associativity=$core->{l2_AC} ";   
    $cache.= "-l15_num_threads=$core->{l15_mshr} " if ($core->{'l15_mshr'} ne '2');
    $cache.= "-hpdc" if (defined $core->{'hpdc_en'} && $core->{'hpdc_en'}==1)  ;
    return $cache;
}

sub log2{
    my $num=shift;
    my $log=($num <=1) ? 1: 0;
    while( (1<< $log)  < $num) {
                $log++;
    }
    return  $log;
}


sub get_cache_conf_name {
    my $core=shift;
    my $l2  = log2(int($core->{l2_SZ}));
    my $l15 = log2(int($core->{l15_SZ}));
    my $l1i = log2(int($core->{l1i_SZ}));
    my $l1d = log2(int($core->{l1d_SZ}));
    my $l15_m = ($core->{'l15_mshr'} eq '2')? "" : log2(int($core->{l15_mshr}));

    my $l2_as  = int($core->{l2_AC});
    my $l15_as = int($core->{l15_AC});
    my $l1i_as = int($core->{l1i_AC});
    my $l1d_as = int($core->{l1d_AC});
    
	my $hpdc="";
	$hpdc = "h" if (defined $core->{'hpdc_en'} && $core->{'hpdc_en'}==1)  ;
    
    my $name = "C${l2}${l15}${l1i}${l1d}${l15_m}${l2_as}${l15_as}${l1i_as}${l1d_as}${hpdc}";
    return $name;


}    

my %generated_bins;

sub run_model {
    my ($core,$mesh_dim,$noc,$mc,$mc_loc,$app,$problem,$mc_lat,$app_loc, $force, $verbus)=@_;
    
    $app_loc = 'Local' if (!defined $app_loc);
    my $name= $app->{name};
    my $loc = "benchmark/app-scalar/$name";  

	my @bin_mpi = ( 
	    "metro_chipset/rel-0.1/obj_dir/Vmetro_chipset",       
        "metro_fake_mem/rel-0.1/obj_dir/Vmetro_fake_mem",        
        "metro_tile/rel-0.1/obj_dir/Vmetro_tile",        
        "metro_chipset/rel-0.1/flist",
        "metro_fake_mem/rel-0.1/flist",
        "metro_tile/rel-0.1/flist",    
    );

    

    my $Dtype = $app->{Dtype};
    my $c = $mesh_dim * $mesh_dim;        
    my $problem_total=problem_cal($problem,$mesh_dim, $app);
    my $compile;    
    my $core_name=$core->{name};
    my $cache= get_cache_conf($core);
    my $bin_name = "${core_name}_${name}${c}_${problem_total}.riscv";
    my $app_name = app_inst_name_gen($problem,$mesh_dim, $app, $app_loc);
    my $app_bin_dir="build/app_bin/${core_name}_${c}_$app_name";
    my $mode_name =gen_model_name($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat);
    my $build_dir= "build/METRO_MPI_ALL/$mode_name";
    my $model_dir= "build/METRO_MPI_ALL/${mode_name}_${app_name}";
    my $stk = $app->{stk};
    $stk *=1024;
    
    if($app_loc eq 'Local') {              
        my @temp = split($piton_root,$mpi_sim_dir);
        my $diff =$temp[-1];
        $loc="$diff/$loc";
        
        my $extra="";
        if (${name} eq 'mt-stream'){
			my $pause = $app->{pause};
			my $ratio = $app->{ratio};
			$extra = "-e \"--pause=${pause} --RD_ratio=${ratio} \"";
		} 
        

        my $gen = "cd ..; bash compile.sh -c $core_name -p $problem_total  -m $c -a $name -d $Dtype -S $stk $extra -D \"-DREPORT_OP_METRICS  -DVERYFY_RESULT\"";        
        $compile= "cd $piton_root/$loc; rm -f bin/*; $gen;  cd $piton_root";
        
    } else {
        # hca server
        my $loc_hca = $apps_hca{$name};
        $compile= "cd $piton_root/$loc; rm bin/*; ssh  hca \" cd ~/risc-v-benchmarks/; source configure settings/x86-hca-server;  cd  $loc_hca; make clean; make bmetal BM_SIZE=$problem_total RVB_BMETAL_VER=$core_name BM_CORES=$c \"; scp hca:$loc_hca/bin/*  $piton_root/$app_bin_dir/$bin_name  ";
    
    }
    
    
   
    
    my $regex = "${mode_name}_${app_name}";
    my $result_dir ="${sim_results_dir}/${mode_name}_${app_name}";
   
    #check if the resuts exsited or not
    #my $zip = Archive::Zip->new();
    #unless ( $zip->read( "$current_dir/results.zip" ) == AZ_OK ) {
    #    die 'Zip archive read error';
    #}
    #my $member2 = $zip->memberNamed( "${mode_name}_${app_name}/fake_uart.log" );
   
   
   
    
    
    if($force){
    	#remove old results
        my $bash="$ssh rm -f $root/$model_dir/{fake_uart.log,sims.log} ";
        my_run($bash,0,$verbus);  
    }
    elsif(check_result_valid_localy ("${mode_name}_${app_name}")){
   		#exsited locally
        print "${mode_name}_${app_name} is already exsited locally. Skip running\n";
        return;
    } elsif (check_result_valid_remotely("${mode_name}_${app_name}")){
    	#exsited remotely skip it
    	print "${mode_name}_${app_name} is already exsited in remote server. Skip running\n";
        return;    
    }
    
    my $bash="rm -rf $result_dir";
    my_run($bash,0,$verbus);    
    print "Run ${mode_name}_${app_name} ....\n";
    

    my ($exsited_locally,$exsited_remotely) = check_model_bin($mode_name);
    if($exsited_remotely==0){
        print "Error: $mode_name is not generated on remote server yet:\n\t1-If you didnt generate the model before, run it again with -b option.\n\t2-If you run the script with model generation enable, check $model_bin_dir/${mode_name}/logs for any possible error\n";
        #copy remote server logs for error checking
        copy_compilation_log_file (${mode_name},$verbus);
        
        return;
    }


    #*****
    #return;
    
    #Compile benchmark
    if(!defined $generated_bins{$app_bin_dir}){
        print "$app_bin_dir/$bin_name is not generated before. Start generating the binary:...\n"  if($verbus);
        $compile .="; $ssh mkdir -p $root/$app_bin_dir; $ssh rm -f $root/$app_bin_dir/$bin_name;  $scp $piton_root/$loc/bin/$bin_name  $server_root/$app_bin_dir/$bin_name;";   
        $compile .=" $scp $mpi_sim_dir/src/rv64_mem_img $server_root/$app_bin_dir/; $ssh chmod +x $root/$app_bin_dir/rv64_mem_img; $ssh $qta cd $root/$app_bin_dir/; ./rv64_mem_img $bin_name ~/scratch/`$ssh whoami`/riscv_install;$qta";
        my_run($compile,0,$verbus);
        $generated_bins{$app_bin_dir}=1;
    } else {
        print "$app_bin_dir/$bin_name is generated before. Skip building the binary!\n"  if($verbus);
    }
   
    my $app_arg    ="\" -precompiled $bin_name  -asm_diag_path ~/$root/$app_bin_dir/ \"  ";

    #make hard link from model_bin seprately for each app
    #my $cmd="$ssh \" mkdir -p $root/$model_dir/bin;";
    my $cmd="$ssh $qta ";
    foreach my $ff (@bin_mpi){
        my $dd = dirname("$root/$model_dir/$ff");
        $cmd.="mkdir -p $dd; ";
        $cmd.="rm -f $root/$model_dir/$ff; ln  $root/$build_dir/$ff $root/$model_dir/$ff; ";    
    }

   # $cmd.="cp $root/$loc/bin/*  $root/$model_dir/bin;\"";
    $cmd.="$qta";
    my_run($cmd,0,$verbus);

    #make a local copy of benchmarks sources for each model 
    #$cmd = "rsync -a   $root/$loc/../* $root/$model_dir/app-scalar/  --exclude={\'*.riscv.dump\',\'*.riscv\',\'dataset.h\'}; wait;";      
    $cmd = get_metro_mpi_command($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat,$model_dir,'run',$app_arg);
    print "$cmd\n" if($verbus==1);
    
    my $count = () = "$mc" =~ /,/g; $count++; #count the number of"," then +1 for custom mapping
    my $mc_num = (($mc_loc eq "all_edges") || 
        ($mc_loc eq "both_y_edges") || 
        ($mc_loc eq "first_y_edge" ))? $mc : $count;
    
    my $c_num = $mesh_dim * $mesh_dim +1 + $mc_num;
    
    my $run_time_hour = 2;
    $run_time_hour += 1 if ($app->{name} eq 'mt-vadd');
    $run_time_hour += 1 if ($noc->{SMART} > 2);
    $run_time_hour += 1 if ($noc->{SMART} > 4);
    

    my_server_run($cmd,$model_dir,$c_num,'run',$verbus,$run_time_hour); 
        
    
    
    #my $member1 = $zip->removeMember( "${mode_name}_${app_name}/fake_uart.log" );
    #$member1 = $zip->removeMember( "${mode_name}_${app_name}/sims.log" );
    #my $dir_member  = $zip->addDirectory( "${mode_name}_${app_name}/" );
    #my $file_member = $zip->addFile( "$result_dir/fake_uart.log", "${mode_name}_${app_name}/fake_uart.log" );
    #$file_member    = $zip->addFile( "$result_dir/sims.log", "${mode_name}_${app_name}/sims.log" );

    # Save the Zip file
    #unless ( $zip->overwrite() == AZ_OK ) {
    #    die 'Zip archive write error';
    #}
}


sub copy_results {
    my ($core,$mesh_dim,$noc,$mc,$mc_loc,$app,$problem,$mc_lat,$app_loc, $force,$verbus)=@_;
    my $mode_name =gen_model_name($core,$mesh_dim,$noc,$mc,$mc_loc,$mc_lat);
    my $app_name = app_inst_name_gen($problem,$mesh_dim, $app, $app_loc);
    my $result_dir ="${sim_results_dir}/${mode_name}_${app_name}";
    my $model_dir= "build/METRO_MPI_ALL/${mode_name}_${app_name}";
    
   
    my $valid = check_result_valid_localy ("${mode_name}_${app_name}");    
    
    my $copy =  !(-f "$result_dir/fake_uart.log") || !(-f "$result_dir/sims.log") ||  $force || ($valid==0);
    
   # my $cmd= "mkdir -p $result_dir;  $scp  $server_root/$model_dir/fake_uart.log   $result_dir/fake_uart.log; $scp  $server_root/$model_dir/sims.log   $result_dir/sims.log; $ssh rm  -f $root/$model_dir/trace_hart_00.dasm ";
    
    my $cmd= "mkdir -p $result_dir;  $scp -r $server_root/$model_dir/{fake_uart.log,sims.log,/script/*}   $result_dir/";
    
    if ($copy){
     my $valid = check_result_valid_remotely("${mode_name}_${app_name}");
     if($valid) {
     	print "Downloading ${mode_name}_${app_name} simulation results from the remote server...\n";
     	my_run($cmd,0,$verbus);
     }else{
     	#the smulation may hangged. just copy the results if they are not existed
     	if(!(-f "$result_dir/fake_uart.log") || !(-f "$result_dir/sims.log")){
     		print "Downloading ${mode_name}_${app_name} simulation results from the remote server...\n";
     		my_run($cmd,0,$verbus);
     	}
    	print " ** [Info], the ${mode_name}_${app_name} simulation hanged or not run!\n";
     }
    }
}


sub save_file {
    my  ($file_path,$text)=@_;
    open my $fd, ">$file_path" or die "could not open $file_path: $!";
    print $fd $text;
    close $fd;    
}

sub load_file {
    my $file_path=shift;
    my $str;
    if (-f "$file_path") {
                
        $str = do {
                local $/ = undef;
                open my $fh, "<", $file_path
            or die "could not open $file_path: $!";
                <$fh>;
        };

    }
    return $str;
}

#############
# object
############

sub object_add_attribute{
    my ($self,$attribute1,$attribute2,$value)=@_;
    if(!defined $attribute2){$self->{$attribute1}=$value;}
    else {$self->{$attribute1}{$attribute2}=$value;}
}

sub object_get_attribute{
    my ($self,$attribute1,$attribute2)=@_;
    if(!defined $attribute2) {return $self->{$attribute1};}
    return $self->{$attribute1}{$attribute2};
}

sub object_add_attribute_order{
    my ($self,$attribute,@param)=@_;
    my $r = $self->{'parameters_order'}{$attribute};
    my @a;
    @a = @{$r} if(defined $r);
    push (@a,@param);
    @a=uniq(@a);    
    $self->{'parameters_order'}{$attribute} =\@a;
}

sub object_remove_attribute_order{
    my ($self,$attribute,$param)=@_;
    my @r=@{$self->{parameters_order}{$attribute}};
    my @n;
    foreach my $p(@r){
        if( $p ne $param) {push(@n,$p)};    

    }
    $self->{parameters_order}{$attribute}=\@n;

}

sub object_get_attribute_order{
    my ($self,$attribute)=@_;
    return unless(defined $self->{parameters_order}{$attribute});
    my @order=@{$self->{parameters_order}{$attribute}};
    return uniq(@order)
}

sub object_remove_attribute{
    my ($self,$attribute1,$attribute2)=@_;
    if(!defined $attribute2){
        delete $self->{$attribute1} if ( exists( $self->{$attribute1}));
    }
    else {
        delete $self->{$attribute1}{$attribute2} if ( exists( $self->{$attribute1}{$attribute2})); ;
    }
}

sub remove_all_white_spaces($)
{
  my $string = shift;
  $string =~ s/\s+//g;
  return $string;
}

sub capture_number_after {
    my ($after,$text)=@_;
    my @q =split  (/$after/,$text);
    #my $d=$q[1];
    my @d = split (/[^0-9. ]/,$q[1]);
    return $d[0];

}

sub extract_uart_results {
    my $in_file=shift;
    my $string = load_file ($in_file);
    if(!defined $string){
        print "Error: Cannot read $in_file\n";
        return;
    }
    my @list = split('--Stats---',$string);
    my @input_odd_entries_only = @list[grep { $_ % 2 == 1 } 0..$#list];
    my $expr=0;
    my %results;
    my @names;
    foreach my $p (@input_odd_entries_only){
        get_acuumulaive_results(\%results,$p,"$uart_perfix",$expr );
        $expr++;
    }       
    return %results;
}

sub  extract_pck_st {
	my ($in_file , $expr_name)=@_;	
	my $string = load_file ($in_file);
    if(!defined $string){
        print "Error: Cannot read $in_file\n";
        return;
    }
	my @list = split('---------Packet Stats-------------',$string);
	my $p = $list[1];
	return if (!defined $p);
	my @lines = split ("\n",$p );
	my $num=0;	
	my $column_names;
	my $row_names="";	
	foreach my $line (@lines) {
    if(length($line)>3 ){
	    if($num==0){
		   $column_names=$line;
		} else{
			$row_names.="$line\n";		 
		}
		$num++;
	 }
	}
	
   my %graphs;
   #create png folder
   my $out_dir="${chart_dir}/expr-${expr_name}/pck_st";
   make_path ("$out_dir") unless(-d $out_dir ); 
   my @l=split('/', $in_file);
   my $n = $l[-2];
   $graphs{file_name}="$out_dir/$n-Run:${expr_name}.png";
   $graphs{X_Title} = "NoC name";
   $graphs{Y_Title} = "#pck";
   $graphs{G_Title} = "Number of injected packets with different packet sizes",
   $graphs{columns} = "$column_names";
   $graphs{rows} = "$row_names";
   make_bar_graph(\%graphs);	   
}


sub parse_timestamp {
    my ($timestamp) = @_;
    return 0 if(!defined $timestamp);
    if ($timestamp =~ /(\d+):(\d+):(\d+)/) {
        my $hours   = $1;
        my $minutes = $2;
        my $seconds = $3;
        return $hours * 60 + $minutes;
    }
    return 0;
}


#results that are accumulated for al cores and represented in two lines
sub get_acuumulaive_results { 
    my ($ref,$input,$perfix,$expr)=@_; 
    return if (!defined $input);
	my @lines = split ("\n",$input );
	return if(!defined  $lines[1] || !defined  $lines[2]);		
	my @keys = split (',', $lines[1] );
	my @vals = split (',', $lines[2] );		 
	for(my $i=0; $i<scalar @keys;$i++){
		$ref->{$expr}{"$perfix".remove_all_white_spaces($keys[$i])}=remove_all_white_spaces($vals[$i]);
	   # print " \$ref $expr $perfix $keys[$i] = $vals[$i] \n" ;
	}   	
	
}


sub extract_simlog_results {
    my ($in_file,$ref)=@_;
    my $string = load_file ($in_file);
    if(!defined $string){
        print "Error: Cannot read $in_file\n";
        return;
    }
    
 
    my @sections=(
    '---------Memory Stats-------------',
    '---------Cache Stats-------------',
    '---------Flit Stats-------------',
    '---------HPM Stats 0-------------',
    '---------HPM Stats 1-------------', 
    '---------HPM Stats 2-------------', 
    '---------HPM Stats 3-------------',
    '---------HPM Stats 4-------------',
    );
    for my $sec (@sections) {
	    my @list = split($sec,$string);
	   
	    if (defined $list[1]) {    
	       my $p = $list[1];
	       my $expr=0;       
	       my @names;
	       my $num=0;
	       my @lines = split ("\n",$p );
	       foreach my $line (@lines) {
	    	#print "$line\n";
		 if(length($line)>3 ){
		    if($num==0){
		        @names = split (',', $line );
		    } else{
		        my @values = split (',', $line );
		        foreach my $f (@names){
		            my $v= remove_all_white_spaces(shift(@values));
		            $f=remove_all_white_spaces($f);
		            $ref->{0}{$f}{$num-1}=remove_all_white_spaces($v);
		            #print "\$ref->{$expr}{$num}{$f}=$v\n";
		        }
		     }
		     #print "**$num :$line\n";
		     $num++;
		  }
	       }
	    }
    }
   # if(defined $list[2]){
        #print $list[2];
        if ($string =~ /\(Passed\!\)/){
            $ref->{0}{"status"} = 0;
        }elsif ($string =~ /\(Failed\!\)/){
            $ref->{0}{"status"} = 1;
        }
        else{
             $ref->{0}{"status"} = 2; #not finished
        }
   # }else{
    #    $results{0}{"status"} = 0;
  #  }
    
    my @list = split('---------Delay model Stats-------------',$string);
    get_acuumulaive_results($ref,$list[1],$lat_perfix,0 );
    
 


	my ($start_timestamp) = $string =~ /sims: sim_start ([^\n]+)/;
	my ($stop_timestamp)  = $string =~ /sims: sim_stop ([^\n]+)/;

 	my ($start_minutes) = parse_timestamp($start_timestamp);
    my ($stop_minutes)  = parse_timestamp($stop_timestamp);
    my $execution_time =($start_minutes!=0 && $stop_minutes!=0) ? $stop_minutes - $start_minutes:0;
    $ref->{0}{"exec_min"} = $execution_time;         
}    

sub get_results_path {
    my($mmc,$mesh_dim,$core,$noc,$enable_mc_lat,$app,$problem,$app_server)=@_;
    my $mc_loc = $mmc->{loc};
    my $mc = ($mmc->{num} eq 'Y'  )? $mesh_dim :
             ($mmc->{num} eq '2Y' )? 2*$mesh_dim:
             $mmc->{num};    
    #get results folder name
    my $model_name = gen_model_name($core,$mesh_dim,$noc,$mc,$mc_loc,$enable_mc_lat);
    my $app_name   = app_inst_name_gen($problem,$mesh_dim, $app, $app_server);
    return "${sim_results_dir}/${model_name}_${app_name}";
}

sub get_exec_time {
    my($mmc,$mesh_dim,$core,$noc,$enable_mc_lat,$app,$problem,$app_server)=@_;

    my $mc_loc = $mmc->{loc};
    my $mc = ($mmc->{num} eq 'Y'  )? $mesh_dim :
             ($mmc->{num} eq '2Y' )? 2*$mesh_dim:
             $mmc->{num};    
    #get results folder name
    my $model_name = gen_model_name($core,$mesh_dim,$noc,$mc,$mc_loc,$enable_mc_lat);
    my $app_name   = app_inst_name_gen($problem,$mesh_dim, $app, $app_server);
    my $r_file = "${sim_results_dir}/${model_name}_${app_name}/fake_uart.log";
                        
    if (-f $r_file) {
        my $r=    extract_exec_time_results ($r_file);
        return ($r,$r_file);
    }else {
        return ("Error",$r_file);
    }
}

sub my_shift {
    my @in = @{$_[0];};
    my $pos = $_[1];
    for(my $i = 0; $i <$pos; $i++){
        shift  @in;
    }
    return @in;
}


my %model;
my %extracts;
my $total_model=0;
my $total_run=0;
sub recursive_loop {
    my @builds=@{$_[0]};
    my @runs=@{$_[1]};
    my @args=@{$_[2]};
    my $is_built= ((defined $_[3]))? $_[3] : 0;
    my $pos1 =((defined $_[4]))? $_[4] : 0;
    my $pos2 =((defined $_[5]))? $_[5] : 0;
    my @b=  my_shift (\@builds, $pos1);
    my @r=  my_shift (\@runs,   $pos2);
    my ($build,$run,$report,$force,$verbus,$help)=@args;

    if (scalar @r == 0) {
        #print "Run\n";        
        my_run_model (\%model,\@builds,\@runs,\@args);                        
        return;
    }
    if (scalar @b == 0) {
        if ($is_built == 0){
            #print "build\n";
            if($build){                
                my_gen_model (\%model,\@args);                
            }
        }
        my $label=$r[0]{label};
        my @sub_run =@{$r[0]{ref}};
        #shift @r;
        $pos2++ if($pos2<  scalar @runs);
        my $index=0;
        foreach my $p (@sub_run){
            $model{$label}{val}=    $p;    
            $model{$label}{index}=    $index;    
            $index++;            
            #print "$p\n";        
            recursive_loop(\@builds,\@runs,\@args,1,$pos1,$pos2);
        }
        return;
    };
    my $label=$b[0]{label};    
    my @sub =@{$b[0]{ref}};
    #shift @b;
    $pos1++ if($pos1<scalar @builds);
    my $index=0;
    if (scalar @sub == 0) {   
        print "Error in experiment input data: No elemnts is passed for $label\n";
        exit 1;        
    }
    foreach my $p (@sub){
        $model{$label}{val}=    $p;
        $model{$label}{index}=    $index;    
        $index++;            
        recursive_loop(\@builds,\@runs,\@args,0,$pos1,$pos2);
    }
}

sub my_gen_model {
    my %model = %{$_[0]};
    my @args  = @{$_[1]};
    my ($build,$run,$report,$force,$verbus,$help)=@args;

    my ($core, $mesh_dim, $noc) = ($model{'cores'}{val}, $model{'mesh_dims'}{val},$model{'nocs'}{val});
    my ($mmc, $enable_mc_lat) = ( $model{'MMCs'}{val}, $model{'MC_LATs'}{val});
    next if( $noc->{SMART} > ($mesh_dim-1));
    my $mc_loc = $mmc->{loc};
    my $mc = ($mmc->{num} eq 'Y'  )? $mesh_dim :
             ($mmc->{num} eq '2Y' )? 2*$mesh_dim:
              $mmc->{num};
    my @edge_ids =get_edge_routers_ids ($mesh_dim,$mesh_dim,$mc_loc);
    my $edges = scalar @edge_ids;
    if(($mc_loc eq "all_edges") || 
        ($mc_loc eq "both_y_edges") || 
        ($mc_loc eq "first_y_edge" )) {
        return if($mc> $edges);
    }
    gen_model ($core,$mesh_dim,$noc,$mc,$mc_loc,$enable_mc_lat,$force,$verbus) if(!$help);
    $total_model++; 
}

my %report;
my %given_names;
use Data::Dumper;
my %speed;


sub get_Descriptive_statistics { # sum min max rsd avg
  my ($name,$hash_ref,$num,$sep,$row,$col ) = @_;
  my %data_set = %{$hash_ref};
  my $r;
  
 
 
  $r = max values %data_set;
  $report{"max_".$name}{$num}{$sep}{$row}{$col}=$r; 
  $report{"max_".$name}{'folder'}=$name;
  
  $r = min values %data_set;
  $report{"min_".$name}{$num}{$sep}{$row}{$col}=$r; 
  $report{"min_".$name}{'folder'}=$name;
  
  $r = mean ( values %data_set);
  $report{"mean_".$name}{$num}{$sep}{$row}{$col}=$r; 
  $report{"mean_".$name}{'folder'}=$name;
  
  #reletive  standard devison
  $r = RSD ( values %data_set);        
  $report{"rsd_".$name}{$num}{$sep}{$row}{$col}=$r; 
  $report{"rsd_".$name}{'folder'}=$name;
  
  $r = sum ( values %data_set);        
  $report{"sum_".$name}{$num}{$sep}{$row}{$col}=$r; 
  $report{"sum_".$name}{'folder'}=$name;

}

sub reset_Descriptive_statistics { # sum min max rsd avg
  my ($name,$num,$sep,$row,$col ) = @_;
  my $r=0;  
  
  $report{"max_".$name}{$num}{$sep}{$row}{$col}=$r;   
  $report{"min_".$name}{$num}{$sep}{$row}{$col}=$r;   
  $report{"mean_".$name}{$num}{$sep}{$row}{$col}=$r;       
  $report{"rsd_".$name}{$num}{$sep}{$row}{$col}=$r;      
  $report{"sum_".$name}{$num}{$sep}{$row}{$col}=$r; 
}


sub my_run_model {
    my %model = %{$_[0]};
    my @builds= @{$_[1]};
    my @runs  = @{$_[2]};
    my @args  = @{$_[3]};
    my ($build,$run,$report,$force,$verbus,$help,$xpr_name)=@args;

    my ($core, $mesh_dim, $noc) = ($model{'cores'}{val}, $model{'mesh_dims'}{val},$model{'nocs'}{val});
    my ($mmc, $enable_mc_lat) = ( $model{'MMCs'}{val}, $model{'MC_LATs'}{val});
    my ($app, $app_server, $problem  ) = ($model{'apps'}{val},$model{'app_servers'}{val},$model{'problems'}{val});
    
    return if( $noc->{SMART} > ($mesh_dim-1));
    my $mc_loc = $mmc->{loc};
    my $mc = ($mmc->{num} eq 'Y'  )? $mesh_dim :
             ($mmc->{num} eq '2Y' )? 2*$mesh_dim:
              $mmc->{num};
    my @edge_ids =get_edge_routers_ids ($mesh_dim,$mesh_dim,$mc_loc);
    my $edges = scalar @edge_ids;
     if(($mc_loc eq "all_edges") || 
        ($mc_loc eq "both_y_edges") || 
        ($mc_loc eq "first_y_edge" )) {
    return if($mc> $edges);
    }
    run_model ($core,$mesh_dim,$noc,$mc,$mc_loc,$app,$problem,$enable_mc_lat,$app_server,$force,$verbus) if ($run && !$help);
    $total_run++ if ($run && !$report); 
    return if ($help);
    return unless ($report);
	
    copy_results ($core,$mesh_dim,$noc,$mc,$mc_loc,$app,$problem,$enable_mc_lat,$app_server,$force,$verbus);

    my $problem_total=problem_cal($problem,$mesh_dim, $app);

    my %comb;
    foreach my $p (@builds){
        my $label=$p->{label};
        $comb{$label}=$p;
    }
    foreach my $p (@runs){
        my $label=$p->{label};
        $comb{$label}=$p;
    }

    my %dims;
    foreach my $m  (sort keys %model){
        
        
        my $role  = $comb{$m}{role};
        $role =  'sep' if ($role eq 'folder'); 
        
        my $apear = $comb{$m}{apear};
        my $index  = $model{$m}{index};
        my $ref  =$model{$m}{val};
        my $v ;
        if (defined $apear){
            my $st = '$v ='.$apear;
            eval ($st);                
        }
        $dims{$role}{coded} = (defined $dims{$role}{coded}) ? "$dims{$role}{coded}"."-$m$index" : "$m$index";
        $dims{$role}{apear} = (defined $dims{$role}{apear}) ? "$dims{$role}{apear}"."-$v" : "$v" if(defined $v);
       
        if ($comb{$m}{role} eq  'folder'){
        	$dims{$role}{folder} =($dims{$role}{folder}) ? "$dims{$role}{folder}"."-$v" : "$v" if(defined $v);
        }
        
    }
    
    foreach my $dim (sort keys  %dims){
        my $coded  = $dims{$dim}{coded};
        my $name   = $dims{$dim}{apear};
        #$given_names{$coded} = $name;
    }
    my $path = get_results_path($mmc,$mesh_dim,$core,$noc,$enable_mc_lat,$app,$problem,$app_server);
    my $fake_uart = "$path/fake_uart.log";
    #print "$fake_uart\n";
    my %results = extract_uart_results($fake_uart);
    
    my $sep = $dims{'sep'}{coded};
    $sep="-" if(!defined $sep);
    my $row = $dims{'row'}{coded};
    my $col = $dims{'column'}{coded};
   
    #labels
    $given_names{$sep}{folder}=$dims{'sep'}{folder};
    $given_names{$sep}{name}=$dims{'sep'}{apear};
    $given_names{$sep}{$row}{name}=$dims{'row'}{apear};
    $given_names{$sep}{$row}{$col}{name}=$dims{'column'}{apear};
    my $r;

    my @exprs=sort keys %results;    
    foreach my $num (@exprs){       
         my $cycle = $results{$num}{$uart_perfix.'cycles'};
        
         
         $report{'cycles'}{$num}{$sep}{$row}{$col}= (defined $cycle) ? $cycle : 0;
         $speed{$num}{$sep}{$col} = $cycle if(!defined $speed{$num}{$sep}{$col} && $cycle!=0);
         $report{'speed-up'}{$num}{$sep}{$row}{$col}=($cycle==0)? 0 : $speed{$num}{$sep}{$col}/$cycle;
        
        
        #print "{$num}{$sep}{$row}{$col}\n";        
      #  my %cycles = %{$results{$num}{'cycles'}};
      #  my %L2_access = %{$results{$num}{'L2_access'}};
      #  my %L2_miss = %{$results{$num}{'L2_mis'}};
       # my %CPI =%{$results{$num}{'CPI'}};  
        #instructions per core
      #  my %instructions ;  
     #   foreach my $p (sort keys %CPI) { 
      #  	if(defined $CPI{$p} && defined $cycles{$p}){
      #  		$instructions{$p} = ($CPI{$p} == 0) ? 0 : $cycles{$p} / $CPI{$p};
      #  	}else{
      #  		$instructions{$p} =0;
      #  	}
     #   }
        #L2_miss_to_access_ratio
      #  my %L2_miss_2_ac;
     #   my %L2_miss_PKI;
     #   foreach my $p (sort keys %L2_miss) {
     #   	if(defined $L2_miss{$p} && defined $L2_access{$p}){
     #   		$L2_miss_2_ac{$p} =($L2_access{$p}==0)? 0 : ($L2_miss{$p} / $L2_access{$p}) * 100;        		
      #  	}else{
     #   		$L2_miss_2_ac{$p} =0;
     #   	}
        	
       # 	if(defined $L2_miss{$p} && defined $instructions{$p}){
      #  		$L2_miss_PKI{$p} = ($L2_miss{$p}==0)? 0 : ($L2_miss{$p} * 1000)/$instructions{$p};
      #  	}else {
       # 		$L2_miss_PKI{$p} =0;
       # 	}       	
       # }
        
             
        #cycles
     #   $r = max values %cycles;
       # $speed{$num}{$sep}{$col} = $r if(!defined $speed{$num}{$sep}{$col} && $r!=0);
       # $report{'cycles'}{$num}{$sep}{$row}{$col}=$r;
      #  $report{'speed-up'}{$num}{$sep}{$row}{$col}=($r==0)? 0 : $speed{$num}{$sep}{$col}/$r;
        
        #L2 access      
     #   get_Descriptive_statistics ( 'L2_access', \%L2_access, $num, $sep, $row, $col);
        #L2_miss    
     #   get_Descriptive_statistics ( 'L2_miss', \%L2_miss, $num, $sep, $row, $col);  
        #L2_miss_2_ac
     #   get_Descriptive_statistics ( 'L2_miss_to_acc', \%L2_miss_2_ac, $num, $sep, $row, $col);  
        #CPI       
     #   get_Descriptive_statistics ( 'CPI', \%CPI, $num, $sep, $row, $col);  
        #instructions per core
    #    get_Descriptive_statistics ( 'instructions', \%instructions, $num, $sep, $row, $col);
        #L2_miss_PKI
     #   get_Descriptive_statistics ( 'L2_miss_PKI', \%L2_miss_PKI, $num, $sep, $row, $col);       
        
    }
    unless (%results){
        my $num =0;
        #cycles
        $report{'cycles'}{$num}{$sep}{$row}{$col}=0;
        #speed_up
        $report{'speed-up'}{$num}{$sep}{$row}{$col}= 0 ;
        #L2 access  
        reset_Descriptive_statistics ("L2_access",$num,$sep,$row,$col ) ;             
        #L2 miss    
        reset_Descriptive_statistics ("L2_miss",$num,$sep,$row,$col ) ; 
        #L2_miss_2_ac
        reset_Descriptive_statistics ( 'L2_miss_to_acc', $num, $sep, $row, $col);  
        #CPI       
        reset_Descriptive_statistics ( 'CPI', $num, $sep, $row, $col);  
        #instructions per core
        reset_Descriptive_statistics ( 'instructions', $num, $sep, $row, $col);  
        #L2_miss_PKI
        reset_Descriptive_statistics ( 'L2_miss_PKI', $num, $sep, $row, $col);        
        
     }


    my $simlog_file = "$path/sims.log";
    extract_simlog_results($simlog_file,\%results );
  
   
    my $num=0;
    #my @caches=('l15_tag_indx', 'l15_dat_indx', 'l2_tag_indx', 'l2_dat_indx', 'l2_access', 'l2_miss' , 'avg_lat');
    my @flit_st=('NoC1_out', 'NoC2_out', 'NoC3_out', 'NoC1_in', 'NoC2_in', 'NoC3_in');
    
    
    my @aa=("1-flit","3-flit","4-flit","9-flit","11-flit","MPI Rank", "MPI Dest Rank","#flit_in","#flit_out","ticks","cycles","MPIRank","Memnum","MPIDestRank","Tilenum");
    my @filter = (@flit_st,@aa);
    my %sts = %{$results{$num}};
    my @all_stats=sort keys %sts;
      
    # Create a hash to store the elements of array @flit_st
    my %lookup = map { $_ => 1 } @filter;
    # Filter elements of array @all_stats  that are not in array @flit_st
    my @filtered_stat = grep { !$lookup{$_} } @all_stats;
    
    
   

    unless (%results){
      reset_Descriptive_statistics ( 'mem_flit_in_percent', $num, $sep, $row, $col); 
      reset_Descriptive_statistics ( 'mem_flit_out_percent', $num, $sep, $row, $col);  
      foreach my $p (@filtered_stat){
    	reset_Descriptive_statistics ( $p, $num, $sep, $row, $col); 
      } 
      foreach my $p (@flit_st){
    	reset_Descriptive_statistics ( $p, $num."_flit", $sep, $row, $col); 
    	reset_Descriptive_statistics ( $p, $num."_flit_per_clk", $sep, $row, $col); 
      } 
      
      return;
    } 
    
    
    foreach my $p (@filtered_stat){
        my $hash_ref = $results{$num}{$p};
        if(defined $hash_ref){
            if (ref($hash_ref) eq 'HASH') {
    	        my %data = %{$hash_ref};
    	        get_Descriptive_statistics ( $p, \%data, $num, $sep, $row, $col); 
    	    }
        }
    } 
    
    foreach my $p (@flit_st){
        if(defined $results{$num}{$p}){
    	    my %data = %{$results{$num}{$p}};
    	    get_Descriptive_statistics ( $p."_flit", \%data, $num, $sep, $row, $col); 
    	    #tile flit counters are gathered for the last region of intreset
    	    my $cycle = $report{'cycles'}{$num}{$sep}{$row}{$col};
    	    foreach my $p (sort keys %data)  {$data{$p} = ($cycle==0)? 0 : $data{$p} / $cycle;}
    	    get_Descriptive_statistics ( $p."_flit_per_clk", \%data, $num, $sep, $row, $col); 
        }
    } 
    
    
    
    if(defined $results{$num}{'cycles'}){
       
		my %cycles = %{$results{$num}{'cycles'}};
		my %flit_in= %{$results{$num}{'#flit_in'}};
		my %flit_out= %{$results{$num}{'#flit_out'}};
		foreach my $p (sort keys %flit_in)  {$flit_in{$p} =  ($flit_in{$p}*100) / (mean values %cycles)}
		foreach my $p (sort keys %flit_out) {$flit_out{$p}=  ($flit_out{$p}*100)/ (mean values %cycles)}
		
		get_Descriptive_statistics ( 'mem_flit_in_percent', \%flit_in, $num, $sep, $row, $col); 
		get_Descriptive_statistics ( 'mem_flit_out_percent', \%flit_out, $num, $sep, $row, $col);		
        
	}else{    
		reset_Descriptive_statistics ( 'mem_flit_in_percent', $num, $sep, $row, $col); 
		reset_Descriptive_statistics ( 'mem_flit_out_percent', $num, $sep, $row, $col);     
	}
    my $status = $results{0}{'status'};
    $report{'status'}{$num}{$sep}{$row}{$col}=$status;
    $report{'simulation time minuts'}{$num}{$sep}{$row}{$col}=$results{0}{'exec_min'};
    
    my @array=("Min","Mean", "Max", "Bandwidth","#rd","#wr","Ratio", "clk", "RD_BW", "WR_BW", "RW_BW");
    foreach my $p (@array){
    	my $ff="$lat_perfix".$p;
    	if (defined  $results{0}{$ff}) {
    	   $report{$ff}{$num}{$sep}{$row}{$col}= $results{0}{$ff};   
    	   $report{$ff}{'folder'}='Lat_Model'; 	  
    	}
    }
    
    @array=("inst_max", "inst_min", "inst_avg", "inst_max_core", "inst_min_core", "avg_CPI");
    foreach my $p (@array){
        my $ff="$uart_perfix".$p;
       
    	if (defined  $results{0}{$ff}) {
    	   $report{$ff}{$num}{$sep}{$row}{$col}= $results{0}{$ff};   
    	   $report{$ff}{'folder'}='Instructions'; 	  
    	}
    } 
    

       
    extract_pck_st ($simlog_file,$xpr_name );


}

sub convert_num {
	my ($n)=@_;	
	my $log=log2($n);
	my $p = 
	  ($log<10)? "" : 
	  ($log<20)? "K":
	  ($log<30)? "M": "G";
	  
	my $Q = 
	  ($log<10)?  1 : 
	  ($log<20)?  1024:
	  ($log<30)?  1024*1024: 1024*1024*1024;  
	  
	my $m = int($n/($Q));
	my $k = int(($n % $Q)/($Q/10));  		
	return "${m}$p" if $k==0;
	return "${m}.${k}$p";
}

sub get_sub_folder_name {
	my $string=shift;
	return undef if(!defined $string) ;
	my $subs ={
		'L2' =>'L2|l2',		
		'L15'=>'l1',
		'Flit'=>'flit',
		'Instruction'=>'CPI|instruct',
		'Branch'=>'branch',
		'Dcache'=>'dcache',
		'Icache'=>'icache',
		'Stall'=>'stall',
		'TLB'=>'tlb',
		'LD_ST'=>'exe_ld|exe_st|ld_st',
		'PTW'=>'ptw',
		'OTHER' =>'depend|list',
		
		
#		'Delay'=>'Mem_Delay'	
	};
	my %k =%{$subs};
	foreach my $s (sort keys %k){	 
		my $p=$k{$s};
	 	return "$s/$string"  if ($string =~ /$p/);
	}
	return $string;
}


# Custom sort function that extracts the numeric part and sorts numerically
sub my_sort {
	my $ref=shift;
	my @sorted_keys = sort {
		my $anum_a = $a;$anum_a=~ s/[^0-9]//g;
		my $anum_b = $b;$anum_b=~ s/[^0-9]//g;		
		if (length $anum_a && length $anum_b) {		
		    return int($anum_a) <=> int($anum_b);
		} else {
		    return $a cmp $b;
		}
	} keys %$ref;
	return @sorted_keys;
}


sub print_results {
   my ($build,$run,$report,$force, $expr_name)=@{$_[0]};
   my %grap_info = %{$_[1]};
   return    if($report==0);

   #create png folder
   my $out_dir="${chart_dir}/expr-${expr_name}";
   make_path ("$out_dir") unless(-d $out_dir );
   #mkdir($out_dir, 0700) unless(-d $out_dir );   
   my %graphs;
   my $sep_name="";
   my $column_names="";
   my $row_names="";
   my $out="";
   my $column_names_final="";
   foreach my $statistic (my_sort(\%report)){        
        foreach my $expr (my_sort($report{$statistic})){  
            next if ($expr eq 'folder') ;         
            foreach my $sep (my_sort($report{$statistic}{$expr})){
                $sep_name="$given_names{$sep}{name}" if(defined $given_names{$sep}{name});
                my $folder_name;
                $folder_name=  "$given_names{$sep}{folder}" if(defined $given_names{$sep}{'folder'});
                $row_names="";               
                foreach my $row (my_sort($report{$statistic}{$expr}{$sep})){              
                    $row_names.="$given_names{$sep}{$row}{name},";
                    $column_names="name,";
                    foreach my $colmn (my_sort($report{$statistic}{$expr}{$sep}{$row})){
                        $column_names.="$given_names{$sep}{$row}{$colmn}{name},";
                        my $r = $report{$statistic}{$expr}{$sep}{$row}{$colmn};
                        $row_names.= "$r,";
                    }
                    $column_names_final = $column_names if (length($column_names_final) < length($column_names));
                    $row_names.="\n";
                }
                
                my $sub_folder =get_sub_folder_name( $report{$statistic}{'folder'});
                if(defined $sub_folder){
                	$folder_name= (defined $folder_name)? "$folder_name/$sub_folder": $sub_folder;
                }                
                if(defined $folder_name){
                	make_path ("$out_dir/$folder_name") unless(-d "$out_dir/$folder_name" );
                	$graphs{file_name}="$out_dir/$folder_name/${statistic}-${sep_name}Run:${expr}.png";
                }else{
                    $graphs{file_name}="$out_dir/${statistic}-${sep_name}Run:${expr}.png";
                    
                    
                }
                $graphs{X_Title} = $grap_info{xtitle};
                $graphs{Y_Title} = "$statistic";
                $graphs{G_Title} = "${sep_name}",
                $graphs{columns} = "$column_names_final";
                $graphs{rows} = "$row_names";
                
                make_bar_graph(\%graphs) if ($grap_info{type} eq 'bar');
                make_line_graph(\%graphs)if ($grap_info{type} eq 'line');

                $out.=
"--------------ST:$statistic--------------
   --${sep_name}-Run:$expr--
$column_names
$row_names


";
            }
        }
   }

	


    my $txt = "$out_dir/${expr_name}.txt";
    save_file ($txt,$out);
    print "Results text are reported in $txt\n";
}

sub run_this_script_on_server{
     $ssh= "cd $piton_root/..;";
     $server_root = $root;
     $scp="cd $piton_root/..; cp";
     $run_on_server=1;
     $qta=""
}


sub run_the_experiment {
    my @builds=@{$_[0]};
    my @runs=@{$_[1]};
    my @args = @{$_[2]};
    my %grap_info;
    if (defined $_[3]){
        %grap_info=%{$_[3]};
    } else{
        $grap_info{xtitle} = undef;
	    $grap_info{type}   = 'bar';
    }
    
    make_path ($sim_results_dir ) unless(-d $sim_results_dir );
    make_path ($model_bin_dir   ) unless(-d $model_bin_dir   );
    my ($build,$run,$report,$force,$name,$help,$verbus,$q,$t)=@args;
    
    
    
    if(defined $q) {$server{'RUN_QUEUE'}=$q  if($q ne 'Default');}
    if(defined $t) {$server{'RUN_TIME' }=$t  if($t ne 'Default');}
    
 
    my @a1 = (1,0,0,$force,$verbus,$help,$name);
    my @a2 = (0,1,0,$force,$verbus,$help,$name);
    my @a3 = (0,0,1,$force,$verbus,$help,$name);
    $total_model=0;
    $total_run=0;
    if($build){
    	recursive_loop (\@builds,\@runs,\@a1); 
        wait_until_squeue_is_empty() if(!$help);
    	get_list_of_all_models_dir_in_server() if(!$help);
    }
    
    if($run) {
        recursive_loop (\@builds,\@runs,\@a2); 
        wait_until_squeue_is_empty() if(!$help);
    } 

    if($report && !$help){
      recursive_loop (\@builds,\@runs,\@a3); 
      print_results(\@args,\%grap_info);
    }
    print "\t\tTotal generated model:$total_model, Total simulation run: $total_run\n"
}

sub make_bar_graph {
    my %graphs_info=%{$_[0]};    
    my $out_file = $graphs_info{file_name};
    my @legend_keys = split(',',$graphs_info{columns});
    shift @legend_keys;
    
    my @rows =split('\n',$graphs_info{rows});
    my %data;
    my $max=0;
    

    foreach my $row (@rows){
        my @a1 = split (',' , $row);
        foreach (my $i=0; $i<scalar @a1; $i++){
        push @{$data{$i}}, $a1[$i];
        if($i!=0){ $max = ($max < $a1[$i])? $a1[$i] : $max;}
        }        
    }
    my @results;
    
    foreach my $p (sort  { $a <=> $b } keys %data ) {
        my @d=@{$data{$p}};
        $results[$p]=\@d;
        
    }

    #my $graph = new GD::Graph::bars(800, 800);
    #$graph->set(
    #    overwrite => 0,
    #    x_label => $graphs_info{X_Title},
    #    y_label => $graphs_info{Y_Title},
    #    title   => $graphs_info{G_Title},
    #    y_max_value => $max+1,
    #    y_tick_number => 18,
    #    y_label_skip => 2,
    #    x_label_skip => 1,
    #    x_all_ticks => 1,
    #    x_labels_vertical => 1,
    #    box_axis => 1,
    #    y_long_ticks => 1,
    #    legend_placement => $graphs_info{legend_placement},
    #    #dclrs=>\@color,
    #    y_number_format=>"%.1f",        
    #    transparent       => '0',
     #      bgclr             => 'white',
    #       boxclr            => 'white',
    #       fgclr             => 'black',
#        textclr          => 'black',
#        labelclr      => 'black',
#        axislabelclr      => 'black',
#        legendclr      =>  'black',
#        bargroup_spacing=>4,
#        #cycle_clrs        => '1',        
#        # Draw bars with width 3 pixels
 #          # bar_width   => 20,
  #      # Separate the bars with 4 pixels
   #     #bar_spacing => 1,
    #    #values_space =>1000,
    #    # Show the grid
    #    #long_ticks  => 1,
    #    # Show values on top of each bar
    #    #show_values => 1,
    #);

    #my $font_file = "$mpi_sim_dir/src/FreeSans.ttf";
    #$graph->set_legend(@legend_keys);
    #$graph->set_title_font($font_file, 20);
    #$graph->set_x_label_font($font_file, 16);
    #$graph->set_y_label_font($font_file, 16);
    #$graph->set_x_axis_font($font_file, 11);
    #$graph->set_y_axis_font($font_file, 11);
    #$graph->set_legend_font($font_file, 15);

    #my $data = GD::Graph::Data->new(\@results) or die GD::Graph::Data->error;
    #my $gd=$graph->plot($data) or warn $graph->error;
    #my $png = $gd->png;
    #save_file($out_file,$gd->png);
    
   	
	


    my $chart = Chart::Gnuplot->new(
        output => "$out_file",
        terminal=> "png enhanced font \"arial,18\"",
        xlabel => {text=>$graphs_info{X_Title},        
            font   => ", 16",
        },
        ylabel => {
            
            text=>$graphs_info{Y_Title},
            font   => ", 16",
        },
        imagesize=>"800,800",
        title  =>{
            text=> $graphs_info{G_Title},
            font   => ", 20",
        },
    
        yrange => "[-0.001:$max+$max*0.05]", # make a 5% gap tp the top
        
        xtics => {
            'rotate'=> '60 right',
            font   => ", 11"
            },
        ytics => {
            #'rotate'=> '60 right',
            font   => ", 11"
            },

        legend => {
#    position => "outside bottom",
#    width    => 3,
#    height   => 4,
#    align    => "left",

    order    => "tmargin horizontal reverse",
        },
        
        bmargin=> 'at screen 0.35',
        key=>"enhanced font  \", 11\" ",
        
        
        #{labelfmt => "%m-%d %H", rotate => -90}
        
    );
    

    my @x;
    my $i=0;
    my @dataset;
    @legend_keys = split(',',$graphs_info{columns});
    foreach my $p (sort  { $a <=> $b } keys %data ) {
        #print "$p:@{$data{$p}}\n";
        my $legend_key =$legend_keys[$p];
        $legend_key =~ s/_/\\\\\\\_/g; #To show under score correctly
        if($p==0){
        	my @xx;
            foreach my $ll (@{$data{$p}}){
            	$ll=~ s/_/\\\\\\\_/g;
            	push (@xx,$ll);
            }
            @x=@xx;
            
            $i++;

        }else{
            my @y=@{$data{$p}};
            for(my $loop_index = 0; $loop_index <= $#x; $loop_index++) {
                $y[$loop_index] = 0 if(!defined $y[$loop_index]);
            }
            $dataset[$p-1]=Chart::Gnuplot::DataSet->new(
                xdata => \@x,
                ydata => \@y,
                title => "$legend_key",   
                fill  => {density => 0.5},
                style => "histograms",
                #color => "blue"
            );
        }
            
    }





    # Raw data

# Plot the graph
$chart->plot2d(@dataset);
print "Result chart is created in $out_file\n";

}



sub make_line_graph {
    my %graphs_info=%{$_[0]};    
    my $out_file = $graphs_info{file_name};
    my @legend_keys = split(',',$graphs_info{columns});
    shift @legend_keys;
    
    my @rows =split('\n',$graphs_info{rows});
    my %data;
    my $max=0;
    

    foreach my $row (@rows){
        my @a1 = split (',' , $row);
        foreach (my $i=0; $i<scalar @a1; $i++){
        push @{$data{$i}}, $a1[$i];
        if($i!=0){ $max = ($max < $a1[$i])? $a1[$i] : $max;}
        }        
    }
    my @results;
    
    my @xx;
    foreach my $p (sort  { $a <=> $b } keys %data ) {
        my @d=@{$data{$p}};
        $results[$p]=\@d;
        if($p==0){
            @xx=@{$data{$p}};
        }       
    }
    my @tiks;
    my $nn=0;
    my @xtmp;
    foreach my $p (@xx) {
        push (@tiks,"\"$p\" $nn");
        push (@xtmp,$nn);
        $nn++;
    }

   




    my $chart = Chart::Gnuplot->new(
        output => "$out_file",
        terminal=> "png enhanced font \"arial,18\"",
        xlabel => {text=>$graphs_info{X_Title},        
            font   => ", 16",
        },
        ylabel => {
            
            text=>$graphs_info{Y_Title},
            font   => ", 16",
        },
        imagesize=>"800,800",
        title  =>{
            text=> $graphs_info{G_Title},
            font   => ", 20",
        },
    
        yrange => "[-0.0001:$max+$max*0.05]", # make a 5% gap tp the top
        
        xtics => {
            'rotate'=> '80 right',
            font   => ", 11",
            labels => \@tiks,
    
            },
        ytics => {
            #'rotate'=> '60 right',
            font   => ", 11"
            },

        legend => {
#    position => "outside bottom",
#    width    => 3,
#    height   => 4,
#    align    => "left",

    order    => "tmargin horizontal reverse",
        },
        
        bmargin=> 'at screen 0.35',
        key=>"enhanced font  \", 11\" ",
        
        
        #{labelfmt => "%m-%d %H", rotate => -90}
        
    );
    

    my @x;
    my $i=0;
    my @dataset;
    @legend_keys = split(',',$graphs_info{columns});
    foreach my $p (sort  { $a <=> $b } keys %data ) {
        #print "$p:@{$data{$p}}\n";
        my $legend_key =$legend_keys[$p];
        $legend_key =~ s/_/\\\\\\\_/g; #To show under score correctly
        if($p==0){
            @x=@{$data{$p}};
            $i++;

        }else{
            my @y=@{$data{$p}};
            for(my $loop_index = 0; $loop_index <= $#x; $loop_index++) {
                $y[$loop_index] = 0 if(!defined $y[$loop_index]);
            }
           # print "@xtmp\n";

            $dataset[$p-1]=Chart::Gnuplot::DataSet->new(
                xdata => \@xtmp,
                ydata => \@y,
                title => "$legend_key",   
                #fill  => {density => 0.5},
                style => "linespoints",
                width => 3,
                #color => "blue"
            );
        }
            
    }





    # Raw data

# Plot the graph
$chart->plot2d(@dataset);
print "Result chart is created in $out_file\n";

}





1;
