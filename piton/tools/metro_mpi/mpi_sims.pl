#!/usr/bin/perl
package GUI;
use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/lib";
use File::Basename;
use Getopt::Long;
use base 'Class::Accessor::Fast';
use Cwd;
use Cwd 'abs_path';
use Getopt::Std;
use List::Util qw(max sum min);
use feature qw(switch);
require "src/mpi_src.pl";


my $start_run = time();


#global parameter arrays:
my @COREs;
my @MESH_DIMs;
my @NOCs;
my @MMCs; 
my @MC_LATs;
my @APPs;
my @APP_LOCs;
my @PROBLEMs;
my @BUILDs;
my @RUNs;
my %GRAPH_SETTING;
my $INFO;



# parameter default
my $Ariane_Default = {name => 'ariane'       ,l2_SZ => '65536' , l2_AC=>4, l15_SZ => '8192' , l15_AC=>4, l1i_SZ => '16384' , l1i_AC=>4,l1d_SZ => '8192' , l1d_AC=>4, l15_mshr=>'2'};
my $OP_NoC        = {noc=>'org'   ,  V=>1,     SSA=>"YES", SMART=>0,    PIPE_REG=>0,    ARBITER=>"RRA", W=>1,    ROUTE=>"XY" , CONCENT=>1, X_MUL=>1, Y_MUL=>1 }; #defaut OP


sub op_noc_width {
    my  ($w1,$w2,$w3)=(shift // 64, shift // 64, shift //64);  
    my $new = {%$OP_NoC, noc1_width => $w1,  noc2_width => $w2, noc3_width => $w3 };    
    return $new;
}


sub k_to_str  { return ($_[0] * 1024) . '' }


sub set_app{
    my ($name,$problem,$pause,$ratio,$stk,$scale);
    ($name,$problem,$pause,$ratio,$stk,$scale)=@_ if($_[0] eq 'mt-stream');
    ($name,$problem,$stk,$scale)=@_ if($_[0] ne 'mt-stream');  
    my $dim = ($name eq 'mt-matmul' || $name eq 'mt-somier') ? 3 : 1;
    my $Dtype = ($name eq 'mt-int-sort' || $name eq 'mt-histogram') ? 'int' : 'double';
    $scale = 'W' if (!defined $scale); # 'S' : 'strong'  'W': weak
    $stk = 128 if(!defined $stk);
    return {name=>$name, problem_dim=>$dim, stk=>$stk, problem_base=>$problem , Dtype=>$Dtype, scale=>$scale,  pause=>$pause, ratio=>$ratio};
}

sub mt_stream       { return set_app ('mt-stream',@_);}


# apear default
my $dim_appr1  = '"Mesh".$ref*$ref';
my $MMC_appr1  = '($ref->{num}==0)? "Baseline" : "M".$ref->{num}';
my $MLAT_appr1 = '$ref->{enable} eq "Yes"? "Lat".$ref->{delay}:"Def"';
my $nocs_appr1 = '$ref->{noc}."S".$ref->{SMART}';
my $nocs_appr2 = '$ref->{noc}."R".$ref->{PIPE_REG}';
my $nocs_appr3 = '$ref->{noc}."S".$ref->{SMART}."R".$ref->{PIPE_REG}';
my $nocs_appr4 = '"CONCENT:".$ref->{CONCENT}';
my $nocs_appr5 = '$ref->{noc1_width}';
my $app_pr_appr1 ='"Size-".convert_num($problem_total)';
my $app_pr_appr2 ='"IPC:".convert_num($app->{problem_base}*$problem)';
my $app_pr_appr3 ='"Stk".$app->{stk}';
my $app_appr1 = '$ref->{name}';
my $core_appr1 = '"$ref->{name}"."-L2(".int($ref->{l2_SZ}/1024)."k)-L15(".int($ref->{l15_SZ}/1024)."K)"';




#########################
#   getopts
##########################

# declare the perl command line flags/options we want to allow
my %options=();
getopts("brchfn:eVq:t:", \%options);

# test for the existence of the options on the command line.
# in a normal program you'd do more than just print these.

# other things found on the command line
print "Other things found on the command line:\n" if $ARGV[0];
foreach (@ARGV)
{
  print "$_\n";
}


if (!keys %options ) {
    $options{h}=1;
    print "Error: You need to provide arguments:\n";
}

if (!defined $options{n} && !defined $options{c} && !defined $options{h} ) {
    $options{h}=1;
    print "Error: You need to specify experiment number using -n flag:\n";
}



if (defined $options{h} ) {

    print "
   Usage: perl mpi_sims.pl -n xpr_name [options]
      -n [expr_num/name] : enter the experiment number you want to run
      -c copy repo to the remote server
      -b only build simulation models
      -r only run simulation models
      -e only extract results
      -f force rebuild/rerun if models/results existed
      -V enable Verbose mode
      -q [queue name] : The name of run quque. The deafult queue is seclted from the server.setting file
      -t [run time] : Run time in form of \"hh:mm:ss\" Default run time is 2 hours.
      -h show this help
   xpr_name:\n";

    my @list = get_list_of_experiments();
    foreach my $p (sort @list){
        $p =~ s/\s+//;
        print "\t [$p] ";
        my $expr_st= "run_experiment_$p (1,1,0,0,\"$p\",1,0)";
        #print "$expr_st;\n";
        unless (eval($expr_st)){
             print "Output captured : $@\n" if (length $@ > 1);
        }
    }

exit;
}


$options{b} = 0 if (!defined $options{b});
$options{r} = 0 if (!defined $options{r});
$options{e} = 0 if (!defined $options{e});
$options{f} = 0 if (!defined $options{f});
$options{h} = 0 if (!defined $options{h});
$options{V} = 0 if (!defined $options{V});
$options{c} = 0 if (!defined $options{c});
$options{q} = "Default"   if (!defined $options{q});
$options{t} = "Default" if (!defined $options{t});




if($options{b}==0 && $options{r}==0 && $options{e}==0 && $options{c} ==0){
    $options{b}=1;
    $options{r}=1;
    $options{e}=1;
}



if ( $options{b}==1 || $options{r}==1) {
    print "get list of all models on the remote server\n";
    get_list_of_all_models_dir_in_server();
}

if ($options{r}==1) {
    print "get all uart results\n";
    get_uart_results();
}

if ($options{c} == 1){
    copy_repo_on_server();
    exit (0) if (!defined $options{n} )
}

my @opts=($options{b},$options{r},$options{e},$options{f},$options{n},$options{h},$options{V});



if(defined $options{n}){
    my ($build,$run,$report,$force, $expr_name, $help,$verbus)=@opts;
    my $expr_st= "run_experiment_$options{n} ($build,$run,$report,$force,\"$expr_name\",$help,$verbus,\"$options{q}\",\"$options{t}\")";
    print "$expr_st;\n";
    unless (eval($expr_st)){
        print "Output captured: $@\n";
        #print "$options{n} is not a valid experimet number\n";
    }

    my $end_run = time();
    my $run_time = $end_run - $start_run;
    my $hour = int($run_time /3600);
    my $min = int(($run_time % 3600) /60);
    my $sec = ($run_time % 60);
    print "script execution time: $hour:$min:$sec\n";
    exit;

}

sub get_list_of_experiments {
    my $file=__FILE__;
    my $str = load_file ($file);
    my @list= $str =~ /sub\s+run_experiment_(.*)\s*\{/g;
    return @list;
}

sub help_function{
    my ($info,$build,$run,$report,$force,$name,$help)=@_;
    $help=0 if (!defined $help);
    if($help){
        print "$info\n";
        return 0;
    }
    return 0;
}




sub default_setting{
    @COREs = ($Ariane_Default);
    @NOCs  = ($OP_NoC);
    @MESH_DIMs = (8);
    @MMCs= ({loc=> "both_y_edges",num=>'4'});
    @MC_LATs = ({enable=>"Yes", delay=>"150"});   
    @APP_LOCs=('Local');
    @PROBLEMs= (8);
    $GRAPH_SETTING{type}   = 'line';
    
    @BUILDs =(
        { label=>'cores', ref=>\@COREs, role=>'row', apear=>undef},
        { label=>'mesh_dims', ref=>\@MESH_DIMs, role=>'row', apear=>undef},
        { label=>'nocs', ref=>\@NOCs, role=>'row', apear=>undef},
        { label=>'MMCs', ref=>\@MMCs, role=>'column', apear=>undef},
        { label=>'MC_LATs', ref=>\@MC_LATs, role=>'column', apear=>undef},
    );
    @RUNs= (
        { label=>'apps', ref=>\@APPs, role=>'column', apear=>undef},
        { label=>'app_servers', ref=>\@APP_LOCs, role=>'column', apear=>undef},
        { label=>'problems', ref=>\@PROBLEMs, role=>'sep', apear=>undef},
    );   
    
}

sub update_default_setting {
    my($default,$update)=@_;
    
    # Create a hash to map labels to elements in the second array
    my %label_to_element;
    foreach my $element (@{$update}) {
        $label_to_element{$element->{label}} = $element;
    }

    # Loop through the first array and update elements if a matching label is found
    for my $element (@{$default}) {
        my $label = $element->{label};
        if (exists $label_to_element{$label}) {
            $element = $label_to_element{$label};
        }
    }

}

sub run_experiment_mmc {
   my @args = @_;
   $INFO = "Run all apps on 64 Mesh custom mmc map";
   default_setting();  
   @MMCs = (
        {loc=> "custom1",num=>"8,15,48,55"}, #names should be unique 
        {loc=> "custom2",num=>"0,7,56,63"},    
    );   
   @APPs = ( mt_stream(256,0,50));
   return if(help_function($INFO,@args));    
   my @runs = ( 
   { label=>'apps', ref=>\@APPs, role=>'row', apear=>'$ref->{name}'},
   { label=>'problems', ref=>\@PROBLEMs, role=>'row', apear=>undef});
   my @builds =({ label=>'MMCs', ref=>\@MMCs, role=>'column', apear=>'$ref->{loc}'});
   update_default_setting(\@RUNs, \@runs);  
   update_default_setting(\@BUILDs, \@builds);     
   run_the_experiment (\@BUILDs,\@RUNs,\@args);
}


#########################
#     experiment memory realtic delay moodel:
#    
#########################

sub setting_for_mem_delay_model{
   my @read_ratio=@_;
   $INFO = "Use realstic delay model on custom stream benchmark with read_ratio @read_ratio on 64 Mesh ";
    default_setting();  
   @APPs=();
   my @pauses= (0,5,10,20,30,60,100,500);
   for my $ratio (@read_ratio){
       for my $n1 (@pauses) {push @APPs, mt_stream(8192,$n1,$ratio);}  
   } 
   @COREs = ($Ariane_Default);
   @MMCs= ({loc=> "both_y_edges",num=>'6'});
   @MC_LATs = ({enable=>"Yes", delay=>"tools/pli/Realistic_lat/curves_src/cxl"});
   @PROBLEMs= (8);
   @BUILDs =(
        { label=>'cores', ref=>\@COREs, role=>'column', apear=>undef},
        { label=>'mesh_dims', ref=>\@MESH_DIMs, role=>'row', apear=>undef},
        { label=>'nocs', ref=>\@NOCs, role=>'row', apear=>undef},
        { label=>'MMCs', ref=>\@MMCs, role=>'column', apear=>undef},
        { label=>'MC_LATs', ref=>\@MC_LATs, role=>'column', apear=>'"xcl"'},
    );
    
    my $app_apr = (scalar @read_ratio > 1)? '"R".$ref->{ratio}."P".$ref->{pause}' : '$ref->{pause}';
    
   @RUNs= (
        { label=>'apps', ref=>\@APPs, role=>'row', apear=>$app_apr},
        { label=>'app_servers', ref=>\@APP_LOCs, role=>'column', apear=>undef},
        { label=>'problems', ref=>\@PROBLEMs, role=>'sep', apear=>$app_pr_appr2},
    );   
   $GRAPH_SETTING{xtitle} = 'Pause';
}

sub run_experiment_d2 {
    my @args = @_;
    setting_for_mem_delay_model(0);    
    return if(help_function($INFO,@args));
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}

sub run_experiment_d26 {
    my @args = @_;
    setting_for_mem_delay_model(26);
    return if(help_function($INFO,@args));    
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}

sub run_experiment_d50 {
    my @args = @_;
    setting_for_mem_delay_model(50);    
    return if(help_function($INFO,@args));
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}

sub run_experiment_d76 {
    my @args = @_;
    setting_for_mem_delay_model(76);
    return if(help_function($INFO,@args));    
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}

sub run_experiment_d100 {
    my @args = @_;
    setting_for_mem_delay_model(100);
    return if(help_function($INFO,@args));    
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}


sub run_experiment_d {
    my @args = @_;
    setting_for_mem_delay_model(100,76,50,26,2);
    
    return if(help_function($INFO,@args));    
    run_the_experiment (\@BUILDs,\@RUNs,\@args,\%GRAPH_SETTING);
}







