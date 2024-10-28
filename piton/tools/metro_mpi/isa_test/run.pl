#!/usr/bin/perl
package GUI;
use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/../lib";
use File::Basename;
use Getopt::Long;
use Cwd;
use Cwd 'abs_path';
use Getopt::Std;
use List::Util qw(max sum min);
use feature qw(switch);

use IO::CaptureOutput qw(capture qxx qxy);


my $enable_vbuild=1;
my $enable_cp_bin=1;
my $enable_parallel_run=1;
my $enable_extract=1;

my $max_sim_per_job=500;



# Variable configurations
my @variable_conf = (
  {'name' => "noc1_w",  'values' => [64,128,256,512]},
 # {'name' => 'noc2_w',  'values' => [64,128,256,512]}
);

# Default configuration
my $default_conf = {
    name => 'default',
    l2_SZ  => '65536',
    l2_AC  =>  4,
    l15_SZ => '8192',
    l15_AC =>  4,
    l1i_SZ => '16384',
    l1i_AC => 4,
    l1d_SZ => '8192',
    l1d_AC => 4,
    mc_loc => "both_y_edges",
    mc_num => '0',
    x_tile => 1,
    y_tile => 1,
    core   => 'ariane',
    noc1_w => 64,
    noc2_w => 64,
    noc3_w => 64,
};




my %options;
getopts("hn:", \%options);

print "Other things found on the command line:\n" if @ARGV;
foreach (@ARGV) {
  print "$_\n";
}

if (!keys %options) {
    $options{h} = 1;
    print "Error: You need to provide arguments.\n";
}

if (!defined $options{n}) {
    $options{h} = 1;
} elsif ($options{n} ne '0' && $options{n} ne '1') {
    print "Error: $options{n} is an invalid value for argument n.\n";
    $options{h} = 1;
}

if (defined $options{h}) {
    print "
   Usage: perl run.pl -n [test_number]
        0: isa_test_list.txt
        1: isa_v_test_list.txt
   ";
    exit 0;
}

my $test_file_list = ($options{n} eq '0') ? 'isa_test_list.txt' : 'isa_v_test_list.txt';
print "$test_file_list is selected as the isa test list file!\n";

my $result_file = ($options{n} eq '0') ? 'result_isa_test.csv' : 'result_isa_v_test.csv';
print "The results will be saved in $result_file\n";


my $piton_root = $ENV{PITON_ROOT};

my $piton_is_set=1;
if (!defined $piton_root){
    $piton_root= abs_path(__FILE__."/../../../../..");    
    $piton_is_set=0;
    print "Warning environment variable PITON_ROOT is not defined. It is set to the following path by this script:\n\t $piton_root \n";    
}


my $sim_dir = abs_path(__FILE__."/..");



# Generate all combinations of variable configurations
my @results;
generate_configurations(\@variable_conf, $default_conf, \@results);


sub generate_configurations {
    my ($conf_array, $default_conf, $results) = @_;

    my $num_vars = scalar @$conf_array;

    # Recursively generate configurations
    recursive_generation(0, {}, $conf_array, $default_conf, $results);

    sub recursive_generation {
        my ($index, $current_conf, $conf_array, $default_conf, $results) = @_;

        if ($index == $num_vars) {
            # All variables have been assigned, add the configuration to results
            push @$results, merge_configurations($default_conf, $current_conf);
            return;
        }

        my $variable = $conf_array->[$index];
        my $var_name = $variable->{'name'};
        my $var_values = $variable->{'values'};

        foreach my $value (@$var_values) {
            $current_conf->{$var_name} = $value;
            recursive_generation($index + 1, $current_conf, $conf_array, $default_conf, $results);
        }
    }
}

sub merge_configurations {
    my ($default_conf, $current_conf) = @_;

    my %merged_conf = %$default_conf;

    foreach my $key (keys %$current_conf) {
        $merged_conf{$key} = $current_conf->{$key};
    }

    my $name = generate_configuration_name( \%merged_conf );
    $merged_conf {'name'} = $name;

    return \%merged_conf;
}

sub generate_configuration_name {
    my ($conf) = @_;

    my @var_names = sort keys %$conf;
    my @var_values;

    foreach my $var_name (@var_names) {
        next if $var_name eq 'name';
        
        push @var_values, "$var_name" . $conf->{$var_name} if($default_conf->{$var_name} ne $conf->{$var_name});
    }
    
    return join("_", @var_values) if scalar (@var_values) ;
    return "default";
}

sub print_configuration {
    my ($conf) = @_;

    print "Name: " . $conf->{'name'} . "\n";

    foreach my $key (keys %$conf) {
        next if $key eq 'name';
        print "$key: " . $conf->{$key} . "\n";
    }

    print "\n";
}



sub create_configuration {
    my ($conf) = @_;
    my $name = $conf->{'name'};
    my $flags="-config_rtl=MINIMAL_MONITORING -config_rtl=PITON_NO_CHIP_BRIDGE ";
    $flags.= "-config_l1i_size=$conf->{l1i_SZ}  ";
    $flags.= "-config_l1i_associativity=$conf->{l1i_AC} ";
    $flags.= "-config_l1d_size=$conf->{l1d_SZ} ";
    $flags.= "-config_l1d_associativity=$conf->{l1d_AC} ";
    $flags.= "-config_l15_size=$conf->{l15_SZ} ";
    $flags.= "-config_l15_associativity=$conf->{l15_AC} ";
    $flags.= "-config_l2_size=$conf->{l2_SZ} ";
    $flags.= "-config_l2_associativity=$conf->{l2_AC} ";   
    $flags.= "-noc1_width=$conf->{noc1_w} ";
    $flags.= "-noc2_width=$conf->{noc2_w} ";
    $flags.= "-noc3_width=$conf->{noc3_w} ";
    $flags.= "-trap_offset=0x80000000 " if ( $test_file_list eq 'isa_v_test_list.txt');  

    print "Name: " . $conf->{'name'} . "\n";

    my $bin ="$piton_root/build/Verilator/$name/manycore/rel-0.1/obj_dir/Vcmp_top";


    my $cmd = "

  rm -f $piton_root/build/Verilator/$name/*
  cd ../
  mkdir $piton_root/build/Verilator/$name
  bash verialtor_run.sh -x $conf->{x_tile} -y $conf->{y_tile} -z $conf->{core} -w build/Verilator/$name  -b -d -e \"$flags\"
  cd $sim_dir
" ;  
    

   system ($cmd) if($enable_vbuild);
   
   unless(-f $bin){
      print "\n\tError $bin is not generated\n";
      exit 1;
   }  
}

sub save_file {
    my  ($file_path,$text)=@_;
    open my $fd, ">$file_path" or die "could not open $file_path: $!";
    print $fd $text;
    close $fd;    
}

 my $core_num=0;
 my %sims;
 my $list="#!/bin/bash
  cd ..\n";

sub create_isa_test_dir {
  my $conf=shift;
  my $name = $conf->{'name'};
  my $file = $test_file_list;
  open my $info, $file or die "Could not open $file: $!";

  my $bin ="$piton_root/build/Verilator/$name/manycore/rel-0.1/obj_dir/Vcmp_top";
  my $work="$piton_root/build/isa_test";

  my $flags = "-trap_offset=0x80000000 " if ( $test_file_list eq 'isa_v_test_list.txt'); 

    while( my $line = <$info>)  { 
        chop($line);
        my $dir =  "$work/${name}/$line";
        print "$dir\n"  if($enable_cp_bin);   
        my $cmd = "rm -rf $dir
mkdir -p $dir/manycore/rel-0.1/obj_dir/
ln $bin $dir/manycore/rel-0.1/obj_dir/Vcmp_top
ln $piton_root/build/Verilator/$name/manycore/rel-0.1/flist  $dir/manycore/rel-0.1/flist"; 

        system ($cmd) if($enable_cp_bin);
        $list.="bash verialtor_run.sh -x $conf->{x_tile} -y $conf->{y_tile} -z $conf->{core} -w build/isa_test/${name}/$line -a \"-precompiled $line.S\" -r -d -e \" $flags  \" &\n"; 
        $sims{"${name}_${line}"}="$piton_root/build/isa_test/${name}/$line";
        $core_num++;
        if($core_num >= $max_sim_per_job){
            craete_runner();
        }        
    }
    close $info;
}


sub remove_all_white_spaces($)
{
  my $string = shift;
  $string =~ s/\s+//g;
  return $string;
}

sub wait_until_squeue_is_empty{
    my @lists;
    print "\nWait for all jobs in slurm queue to finish\n";
    do{
        
        my $bash="squeue -u `whoami`";
        my $id = run_cmd_message_dialog_errors($bash);            
                
        @lists = split("\n",$id);
        my @names;
        my $num=0;
        my %sq;
        my %st;

        foreach my $line (@lists){
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
        my $timestamp = localtime(time);
        print "    $timestamp    Total jobs: $t   ";
        foreach my $p (sort keys %st) {
            print "$p: $st{$p}  "
        }
        print "\n";

        sleep 10;
    
    }while(defined $lists[1]);
}


sub run_cmd_in_back_ground_get_stdout
{
    my $cmd=shift;
     
    STDOUT->flush();
    STDERR->flush();
    
    my  ($stdout, $stderr,$success, $exit) = qxx( $cmd );
    return ($stdout,$exit,$stderr);    
}        


sub run_cmd_message_dialog_errors{
    my ($cmd)=@_;
    my ($stdout,$exit,$stderr)=run_cmd_in_back_ground_get_stdout($cmd);
    if(length $stderr>1){            
        print "Error : $stderr\n";
        #exit 1;
        #return 1;
    }if($exit){
        print "Error : $cmd failed: $stdout\n";
        #return 1;        
    }
    $stdout = "" if (!defined $stdout);
    return     $stdout
    
}

my $job_num=0;
sub craete_runner {
    $list.="wait\n";
    save_file("./run${job_num}.sh",$list); 
    $list="#!/bin/bash\n   cd ..\n";
    my $node_num = int ($core_num/48);
    $node_num++;

    my $sbash="#!/bin/bash
#SBATCH --job-name=Job$job_num
#SBATCH --output=r${job_num}.out
#SBATCH --error=r${job_num}.err
#SBATCH --nodes=$node_num
#SBATCH --cpus-per-task=1
#SBATCH --ntasks=$core_num
#SBATCH --tasks-per-node=48
#SBATCH --qos=debug
#SBATCH --time=0:10:00

mkdir -p ./out;
time bash ./run${job_num}.sh > ./out/${job_num}.log
";

    save_file("./Queue_build${job_num}.sh",$sbash); 
    system ("chmod +x ./Queue_build${job_num}.sh");
    $job_num++;
    $core_num=0;
}


sub extract_results{
    my $failed=0;
    my $passed=0;
    unlink $result_file;
    my $fh;
    open( $fh, '>', $result_file) or die "Could not open file: $result_file $!";
    print "Check simulation results. it may takes few minutes!...\n";
    foreach my $p (sort keys %sims){
        my $log = "$sims{$p}/status.log";       
        if (`grep 'Simulation -> PASS (HIT GOOD TRAP)' $log`) {           
            print $fh "$p, PASS\n";   
            $passed++;       
        } else {
            print $fh "$p, FAILED\n";
            print "ERROR :   $p, FAILED\n";           
            $failed++;
        }   
    }
    close($fh);
    print "\n\t**All simulations (total of $passed) passed successfully!**\n" if($failed==0);
    print "\n\t**Error: Total of $failed simulations failed. The rest of $passed simulation passed successfully!**\n" unless($failed==0);
}


foreach my $result (@results) {
    create_configuration($result);   
}

foreach my $result (@results) {   
    create_isa_test_dir($result);   
}

craete_runner();  

if ($enable_parallel_run){
    for (my $job=0;$job<$job_num;$job++){
        system( "sbatch Queue_build${job}.sh"  );
    }
    wait_until_squeue_is_empty();
}

exit 0 if($enable_extract==0) ;


extract_results();



exit 0;