#!/usr/bin/perl 
package GUI;



#add home dir in perl 5.6
use FindBin;
use lib $FindBin::Bin;
use constant::boolean;


use strict;
use warnings;


use lib 'lib';
require "widget3.pl";
require "./src/pck_type.pl";


use File::Basename;
use File::Copy;
use Getopt::Long;
use base 'Class::Accessor::Fast';


my @hpms=(
"branch_miss",
"is_branch",
"branch_taken",
"exe_store",
"exe_load",
"icache_req",
"icache_kill",
"stall_if",
"stall_id",
"stall_rr",
"stall_exe",
"stall_wb",
"icache_miss_l2_hit",
"icache_miss_kill",
"icache_busy",
"icache_miss_time",
"load_store",
"data_depend",
"struct_depend",
"grad_list_full",
"free_list_empty",
"itlb_access",
"itlb_miss",
"dtlb_access",
"dtlb_miss",
"ptw_buffer_hit",
"ptw_buffer_miss",
"itlb_stall",
"dcache_stall",
"dcache_stall_refill",
"dcache_rtab_rollback",
"dcache_req_onhold",
"dcache_prefetch_req",
"dcache_read_req",
"dcache_write_req",
"dcache_cmo_req",
"dcache_uncached_req",
"dcache_miss_read_req",
"dcache_miss_write_req",
"stall_ir",
"l2_miss",
"l2_access",          
"l15_miss",
"l15_access"
);

my $red_out ="| grep -E --color \"\\b(Error)\\b|\$\"";
use Cwd;
my $current_dir = getcwd;
my $ref_squeue_now =0;

my $piton_root = $ENV{PITON_ROOT};
if (!defined $piton_root){
	$piton_root= abs_path(__FILE__."/../../../..");	
	print "Warning environment variable PITON_ROOT is not defined. It is set to the following path by this script:\n\t $piton_root \n";	
}




my $tmp1 = "$piton_root/build/tmp1.sh";
my $tmp2 = "$piton_root/build/tmp2.sh";
my $lat_curve_dir = "tools/pli/Realistic_lat/curves_src";
my $questa_sh="$current_dir/src/Questasim";
my $rvvo_dir = "$ENV{HOME}/scratch/" . `whoami`;
chomp($rvvo_dir);  # To remove the newline character from the `whoami` output





my $obj_server;
get_server_id();
my $MODULES;
my $modules_ref = $obj_server->object_get_attribute('MODULES');

if(defined $modules_ref){
	my %hash = %{$modules_ref};
	$MODULES="";
	foreach my $p (sort  {$a <=> $b} (keys %hash)){
	  $MODULES.="$hash{$p} "
	}	
}





use Gtk3;

Gtk3->init;

__PACKAGE__->mk_accessors(qw{
    GUI_WORK
    });
my $self= __PACKAGE__->new();
        
set_gui_status($self,"ideal",0);    

sub get_ssh_cmd {
    my ($uname) =get_server_id();	
	my $sim = $self->object_get_attribute('CTRL','SIM') // "Metro-MPI";
	my $ssh =($sim eq 'Questasim')? "ssh -X -q ${uname}" : "ssh  -q ${uname}";
	return $ssh;
}

my @apps = (
{name=>'hello_world_token.c', psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'hello_world_many.c',  psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'hello_world.c',       psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'accu_test.c',         psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>' lrsc_test.c',        psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'add_shared_var.c',    psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'vectorial_mem_access_test.c', psize_def=>undef,  psize=>undef,        precomp=>undef},
{name=>'amo_align.c ',        psize_def=>undef,  psize=>undef,        precomp=>undef},         
{name=>'clint_plic_access.c', psize_def=>undef,  psize=>undef,        precomp=>undef}, 
{name=>'lrsc_single.c',       psize_def=>undef,  psize=>undef,        precomp=>undef},



{name=>'mt-hello',            psize_def=>undef,  psize=>undef,        precomp=>1 },
{name=>'mt-stream',           psize_def=>4096,   psize=>"1,100000,1", precomp=>1, pause=>0, ratio=>0, loop=>1},
);

my $princeton_dir = "$piton_root/piton/verif/diag/assembly/princeton";
opendir(DIR, $princeton_dir) || die "can't opendir $princeton_dir: $!";
my @sparc_apps = grep { /\.s$/ && -f "$princeton_dir/$_" } readdir(DIR);
closedir DIR;

my $sparc_app_list =join (',',  sort @sparc_apps);

my $app_list="";
foreach my $d (@apps) {
    $app_list.= (!defined $app_list)? "$d->{name}" : ",$d->{name}";
}

#make bmetal SIZE=2048 RVB_BMETAL_VER=ariane CORES=2


sub gen_mc_map {

	#get x, y dimention sizes
	my $x=$self->object_get_attribute('CTRL','X');
	my $y=$self->object_get_attribute('CTRL','Y');
	
	my $T1 = $self->object_get_attribute('noc_param','T1');
	my $T2 = $self->object_get_attribute('noc_param','T2');
	my $T3 = $self->object_get_attribute('noc_param','T3');
	if(defined $T1){
		$x= $T1;
		$y= $T2;		
	}
	
	
	
	my $table=def_table($x,$y);
	
	for (my $i=0; $i<$x; $i++){
		for (my $j=0; $j<$y; $j++){
			my $id= $j*$x+$i;
			my $f=$self->object_get_attribute('CTRL',"mc_$id" );		
			my $en=1;
			
			
			if(($i!=0 && $i!=$x-1 && $j!=0 && $j!=$y-1)  ){ # not an edge disable the checkbox
				$self->object_add_attribute('CTRL',"mc_$id","0");
				#$table->attach (gen_colored_label(' ',17),5,10,$j*4,$i*4,'fill','shrink',2,2);	
				#$en=0;
			}else {
				#my $endp = get_mc_endpid($i,$j);
				
			    	add_param_widget  ($self, $id,"mc_$id" , "0", "Check-box", 1, undef, $table,$j*4,$i*4,1, 'CTRL', undef, undef, undef);
			    
			}
			
		}
	}

	return $table;
}

sub get_mc_endpid {
	my ($i,$j)=@_;
	my $piton_X=$self->object_get_attribute('CTRL','X');
	my $piton_Y=$self->object_get_attribute('CTRL','Y');
	my $endp = $piton_X * $piton_Y;
	if ($i == 0 && $j != 0){ 
		$endp = $endp + $piton_X + $piton_X + $j;
	}elsif ($j == $piton_Y-1){ #  and i != 0
        $endp = $endp + $piton_X + $i;      
    }elsif ($i == $piton_X-1){ # and j != PITON_Y_TILES-1
        $endp = $endp + $piton_X + $piton_X + $piton_Y + $j;        
    }elsif ($j == 0){ # and i != PITON_X_TILES-1
        $endp = $endp + $i;
	}else {
	   die "invalid $i,$j\n" ;
	}  
	return $endp;     
}           


sub get_custom_locs{
	my ($self,$name)=@_;
	my $x=$self->object_get_attribute('CTRL','X');
	my $y=$self->object_get_attribute('CTRL','Y');
	my $list;
	my $n=0;
	for (my $j=0; $j<$y; $j++){
		for (my $i=0; $i<$x; $i++){		
			my $id= $j*$x+$i;
			my $t=$self->object_get_attribute('CTRL',"${name}_$id");
			$t='1\'b0' if(!defined $t);
			if($t eq '1\'b1' ){
				$list=(defined $list)? "$list,$id" : "$id";
				$n++;
			}
			
		}
	}
	return ($n,$list);
}

my $setting;
sub get_setting_object {    
    my $paths_file= "$current_dir/server.setting";	
    return ($setting,$paths_file) if(defined $setting);
	__PACKAGE__->mk_accessors(qw{
	LOCAL_SETTING
	});
	if (-f 	$paths_file ){
		$setting= do $paths_file;
	}else{
		$setting = __PACKAGE__->new();		
	}
    return ($setting,$paths_file);
}

sub run_server_setting {
	
	my $set_win = def_popwin_size(50,50,"remote server setting",'percent');
	my $server_dir="$current_dir/servers";
	# Open the directory
    opendir(my $dh, $server_dir) or die "Cannot open directory: $!";
    opendir(DIR, $server_dir) || die "can't opendir $server_dir: $!";
    my @servers = grep { -f "$server_dir/$_" } readdir(DIR);
    closedir DIR;    
    my $servers_list=join (',', @servers); 
    $servers_list=" ,".$servers_list;
	
	my ($obj,$paths_file)= get_setting_object();	
	my $ok = def_image_button('icons/select.png','OK');
	my $table=def_table(10,10,FALSE);
	my $mtable=def_table(10,10,FALSE);	
	my $scrolled_win= add_widget_to_scrolled_win($table);
	$mtable->attach_defaults($scrolled_win,0,1,0,9);
	
	my ($col,$row)=(0,0); my $w;
	($row,$col,$w)=add_param_widget  ($self, "Selecet Server", "SERVERS", "Server settings are located in $server_dir", "Combo-box", $servers_list, undef, $table,$row,$col,1, 'SERVER', 0, undef, 'vertical');
	my $unames = $obj->object_get_attribute('SERVER','UNAMES');
	($row,$col)=add_param_widget  ($obj, "Username", "UNAME", "user name in server e.g bsc***\@mn1.bsc.es", "Combo-box", $unames, undef, $table,$row,$col,1, 'SERVER', 0, undef, 'vertical');
	
	my $queues = $obj->object_get_attribute('SERVER',"QUEUES");	
	if(defined $queues){	
	($row,$col)=add_param_widget  ($obj, "Bulid queue", "BUILD_QUEUE", undef, "Combo-box", $queues, undef, $table,$row,$col,1, 'SERVER', 0, undef, 'vertical');
	($row,$col)=add_param_widget  ($obj, "Run queue", "RUN_QUEUE", undef, "Combo-box", $queues, undef, $table,$row,$col,1, 'SERVER', 0, undef, 'vertical');
	}
	
	my $verilators = $obj->object_get_attribute('VERILATOR','VERSIONS');	
	($row,$col)=add_param_widget  ($obj, "Verilator", "DEFAULT", undef, "Combo-box", $verilators, undef, $table,$row,$col,1, 'VERILATOR', 0, undef, 'vertical');	
	
	
	my $copy  = def_image_button('icons/upload.png');
	set_tip($copy, "Upload repo code to the remote server");
	$table->attach  ( gen_label_in_left("Upload OpenPiton repo") , 0, 2,  $row,$row+1,'fill','shrink',2,2); 
	$table->attach($copy,2,3,$row,$row+1,'fill','shrink',2,2);$row++;
	$copy-> signal_connect("clicked" => sub{ 
		copy_repo_on_server_in_xterm( );
	});
	my @bash_sel; 
	my $r= $obj->object_get_attribute('SBATCHS_EN');
	@bash_sel = @{$r} if (defined $r);

	my $sbatch_ref =$obj->object_get_attribute('SBATCH');    
    if(defined $sbatch_ref){
	    my %hash = %{$sbatch_ref};
	    ($col,$row)=(6,0);
	    $table->attach  ( gen_label_in_left("    Select (S)Bash initial commands   ") , 7, 12,  $row,$row+1,'fill','shrink',2,2); $row++;
	    foreach my $p (sort  {$a <=> $b} (keys %hash)){
	      my $w= gen_checkbutton();
	      $table->attach  ( $w , 8, 9,  $row,$row+1,'fill','shrink',2,2);
	      if (grep { $_ == int($p) } @bash_sel ){
	         $w->set_active(TRUE);
	      }else {
	         $w->set_active(FALSE);	      
	      }
	      $w-> signal_connect("toggled" => sub{
	         if($w->get_active()) {push (@bash_sel, int($p));  }
			 else {@bash_sel=remove_scolar_from_array(\@bash_sel,int($p))}	
			 $obj->object_add_attribute('SBATCHS_EN',undef,\@bash_sel);		
	      });
	      $table->attach  ( gen_label_in_left(" $p-$hash{$p}") , 9, 12,  $row,$row+1,'fill','shrink',2,2); $row++;
	     
	    }	
	    
	    add_Vsep_to_table($table,6,0,$row);
    }	
	
	
	
	$ok->signal_connect("clicked"=> sub{
		#save setting
		open(FILE,  ">$paths_file") || die "Can not open: $!";
		print FILE Data::Dumper->Dump([\%$obj],['glob']);
		close(FILE) || die "Error closing file: $!";	
		$set_win->destroy;
	});
	
		
	$mtable-> attach ($ok , 0, 1,  9, 10,'expand','shrink',2,2); 
	
	$set_win->add ($mtable);
	
	$set_win->show_all;
	
	$w->signal_connect("changed"=> sub{
	  my $newserver =$self->object_get_attribute("SERVER","SERVERS");	
	  return if $newserver eq " ";
	      
	  copy ("$current_dir/servers/$newserver","$current_dir/server.setting" );
	  $setting=undef;	 
	  $set_win->destroy;   
	  run_server_setting();
	  	  
	});
}

sub get_verilator_setting {
    my ($obj,$paths_file)= get_setting_object();
    my $ver = $obj->object_get_attribute('VERILATOR','LOCAL_VER');
	my $HOME =$ENV{HOME};
	my $username = $ENV{USER};
	my $Vname = $ver;
    $Vname =~ s/\./_/g;
    my $verilator_bin  = "$HOME/scratch/$username/verilator_$Vname";
    my $verilator_repo = "$HOME/scratch/$username/verilator_repo";
    return ($ver,$verilator_bin,$verilator_repo);
}

sub get_local_init_setting {
    my ($obj,$paths_file)= get_setting_object();
    my $init="";
    my $ver = $obj->object_get_attribute('QUESTASIM','LOCAL_VER');
    $init.="source $questa_sh/$ver.sh;" if(defined $ver);
    $ver = $obj->object_get_attribute('RVVO','LOCAL_VER');
    $init.="export RISCV=$rvvo_dir/$ver;";
    return $init;   
}


sub run_local_setting {	

	my $set_win = def_popwin_size(50,50,"Local setting",'percent');
	my ($obj,$paths_file)= get_setting_object();
	my $ok = def_image_button('icons/select.png','OK');
	my $table=def_table(10,10,FALSE);
	my $mtable=def_table(10,10,FALSE);	
	my $scrolled_win= add_widget_to_scrolled_win($table);
	$mtable->attach_defaults($scrolled_win,0,1,0,9);	
	my ($col,$row)=(0,0);	
	my $verilators = $obj->object_get_attribute('VERILATOR','VERSIONS');	
	($row,$col)=add_param_widget  ($obj, "Verilator", "LOCAL_VER", undef, "Combo-box", $verilators, undef, $table,$row,$col,1, 'VERILATOR', 0, undef, 'vertical');	
	
	
	
	# get list of Questasims
    opendir(DIR, $questa_sh) || die "can't opendir $questa_sh: $!";
    my @Quetsas = grep { -f "$questa_sh/$_" } readdir(DIR);
    @Quetsas = map { s/\.[^.]+$//r } @Quetsas;
    closedir DIR;    
    my $Quetsa_list=join (',', @Quetsas); 
	($row,$col)=add_param_widget  ($obj, "Questasim", "LOCAL_VER", undef, "Combo-box", $Quetsa_list, undef, $table,$row,$col,1, 'QUESTASIM', 0, undef, 'vertical');	
	
	
	#get list of rvvos
    opendir(DIR, $rvvo_dir ) || die "can't opendir $rvvo_dir: $!";
    my @RVVOs = grep { -d "$rvvo_dir/$_" &&  /^riscv_install/ } readdir(DIR);
    closedir DIR;    
    my $RVVO_list=join (',', @RVVOs); 
	($row,$col)=add_param_widget  ($obj, "RVVO", "LOCAL_VER", undef, "Combo-box", $RVVO_list, undef, $table,$row,$col,1, 'RVVO', 0, undef, 'vertical');	
	
	
	
	$ok->signal_connect("clicked"=> sub{
	    #save setting
		open(FILE,  ">$paths_file") || die "Can not open: $!";
		print FILE Data::Dumper->Dump([\%$obj],['glob']);
		close(FILE) || die "Error closing file: $!";	
	    
	    #for local setting check if selected verilator version existed in system
	    my ($ver,$verilator_bin,$verilator_repo) = get_verilator_setting();
        unless (-d $verilator_bin) {          
            my $response = yes_no_dialog(" Verilator $ver is not installed locally on this machine. Do you want to install it now?");
            if ($response eq "yes") {            
               my $cmd = "bash $current_dir/src/verilator_build.sh -v $ver " ;
               run_in_xterm($cmd);
            }            
        }		
		$set_win->destroy;

	});		
	$mtable-> attach ($ok , 0, 1,  9, 10,'expand','shrink',2,2); 	
	$set_win->add ($mtable);	
	$set_win->show_all;
}




sub load_func {
    my $save_dir="$piton_root/build/my_config";
    my $file;
    my $dialog =  gen_file_dialog (undef, 'cnf');	
    $dialog->set_current_folder ("$save_dir")    ;
	if ( "ok" eq $dialog->run ) {
        	$file = $dialog->get_filename;
        	my ($name,$path,$suffix) = fileparse("$file",qr"\..[^.]*$");
        	if($suffix eq '.cnf'){
            my ($pp,$r,$err) = regen_object($file );
            if ($r){        
                message_dialog("**Error: cannot open $file file: $err\n",'error');
                $dialog->destroy;
                return;
            } 
			clone_obj($self,$pp);
			set_gui_status($self,"ref",1);                   
        	}                                      
		}
     $dialog->destroy;    

}



sub get_ctrl_btm{

	my $target=$self->object_get_attribute('CTRL',"TARGET");
	my $build = def_image_button('icons/select.png','Build');
	my $run   = def_image_button('icons/select.png','Run');
	my $brun  = def_image_button('icons/select.png','Build&run');
	my $save_dir="$piton_root/build/my_config";
	system("mkdir -p $save_dir") unless (-d $save_dir);
	my ($entrybox,$entry) = gen_save_load_widget 
     (
        $self, #the object 
        "config name",#the label shown for setting configuration
        "config_name",#the key name for saveing the setting configuration in object 
        "",#the label full name show in tool tips
        undef,#Where the generted RTL files are loacted. Undef if not aplicaple       
        $save_dir,#where the current configuration seting file is saved
        "cnf",#the extenstion given for configuration seting file
        \&load_func,
        "Save current simulation configuration setting"
        );
	
	
#	my $load  = def_image_button('icons/browse.png','Load');
#	my $entry=gen_entry_object($self,'config_name',undef,undef,undef,undef);
#	my $entrybox=gen_label_info(" Config name:",$entry);
#    my $save      = def_image_button('icons/save.png');	
#    my $open_dir  = def_image_button('icons/open-folder.png');
#    set_tip($save, "Save current simulation configuration setting");
#	#set_tip($open_dir, "Open target setting directory");    	
#	$entrybox->pack_start( $save, FALSE, FALSE, 0);
#	#$entrybox->pack_start( $open_dir , FALSE, FALSE, 0);


	
	my $table=def_table(1,1);
	 
	$table->attach($build,1,2,0,1,'expand','fill',2,2); 
	$table->attach($run,2,3,0,1,'expand','fill',2,2);
	$table->attach($brun,3,4,0,1,'expand','fill',2,2) if($target eq 'Local');
	
	#$table->attach($load,4,5,0,1,'expand','fill',2,2);
	$table->attach($entrybox,5,6,0,1,'expand','fill',2,2);
	
	
	
	$run-> signal_connect("clicked" => sub{ 
		run_bash('run');
	}	);

	$build-> signal_connect("clicked" => sub{		
		run_bash('build');
	}	);


	$brun-> signal_connect("clicked" => sub{ 
		run_bash('build&run');
	}	);

#	my $save_dir="$piton_root/build/my_config";
#	$save-> signal_connect("clicked" => sub{ 
 #   	my $name=$self->object_get_attribute('config_name');
 #   	return  if ( !defined $name);
#		return  if ( length $name == 0);
    	
#		system("mkdir -p $save_dir");
#	    open(FILE,  ">$save_dir/$name.setting") || message_dialog("Can not open $save_dir/$name.setting: $!",'error'); 
#		print FILE Data::Dumper->Dump([\%$self],["config"]);
 #   	close(FILE) || message_dialog("Can not close $save_dir/$name.setting: $!",'error'); 
  #  	message_dialog("Current configuration  \"$name\" is saved as $save_dir/$name.setting.");
    
  #  });

#	$load-> signal_connect("clicked" => sub{ 
		
 #       my $file;
  #  	my $dialog =  gen_file_dialog (undef, 'setting');	
   # 	$dialog->set_current_folder ("$save_dir")    ;
	#    if ( "ok" eq $dialog->run ) {
     #   	$file = $dialog->get_filename;
      #  	my ($name,$path,$suffix) = fileparse("$file",qr"\..[^.]*$");
       # 	if($suffix eq '.setting'){
        #    my ($pp,$r,$err) = regen_object($file );
         #   if ($r){        
          #      message_dialog("**Error: cannot open $file file: $err\n",'error');
           #     $dialog->destroy;
            #    return;
            #} 
#			clone_obj($self,$pp);
#			set_gui_status($self,"ref",1);                   
 #       	}                                      
	#	}
     #$dialog->destroy;    
	#});
	
	return $table
}


sub gen_build_tab {
	my $table=def_table(1,1);
	my $target=$self->object_get_attribute('CTRL',"TARGET");
	$target = "Local" if(!defined $target);	
	my ($obj)= get_setting_object();
	my $supported_sims_server = $obj->object_get_attribute('SERVER',"SIMULATORS");
	my $sims = $target ne "Local" ? $supported_sims_server :"Questasim,Verilator,Metro-MPI";	

	my @info = (
	{ label=>" model name: ", param_name=>'MODEL', type=>"Entry", default_val=>"model1", content=>undef, info=>'Define a unique name for each model.', param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}, 
	{ label=>" Simulator: ", param_name=>'SIM', type=>"Combo-box", default_val=>undef, content=>$sims, info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" x_tiles: ", param_name=>'X', type=>"Spin-button", default_val=>1, content=>"0,16,1", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" y_tiles: ", param_name=>'Y', type=>"Spin-button", default_val=>1, content=>"0,16,1", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" Tile: ", param_name=>'TILE', type=>"Combo-box", default_val=>'ariane', content=>"ariane,lite,sarg,ost1,lox", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	#{ label=>" L2_size: ", param_name=>'L2_SIZE', type=>"Combo-box", default_val=>'65536', content=>"65536,131072,262144,524288,1048576", info=>"L2 cache size per core", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	#{ label=>" L15_size: ", param_name=>'L15_SIZE', type=>"Combo-box", default_val=>'8192', content=>"8192,16384,32768,65536,131072", info=>"L1.5 cache size per core", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" NoC 1 Width: ", param_name=>'NOC1_WIDTH', type=>"Combo-box", default_val=>'64', content=>"64,128,256,512", info=>"NoC 1 bus width in bits", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" NoC 2 Width: ", param_name=>'NOC2_WIDTH', type=>"Combo-box", default_val=>'64', content=>"64,128,256,512", info=>"NoC 1 bus width in bits", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	{ label=>" NoC 3 Width: ", param_name=>'NOC3_WIDTH', type=>"Combo-box", default_val=>'64', content=>"64,128,256,512", info=>"NoC 1 bus width in bits", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
    { label=>" Traffic Visualizer ", param_name=>'MANY_GUI', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Enabled", info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=>undef, loc=>'vertical'},
    { label=>" rtl_timeout: ", param_name=>'RTLTIM', type=>"Spin-button", default_val=>100000000, content=>"0,100000000000,1", info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'},    
    #{ label=>" HBM RD latency: ", param_name=>'HBM_LAT', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Fix-lat,Realistic", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> 'ref_ctrl', loc=>'vertical'},    
	#{ label=>" Multiple memory controllers ", param_name=>'MULTIMC', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Custom,ACME", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref', loc=>'vertical'},
 	{ label=>" HPDC ", param_name=>'HPDC', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Enabled", info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=>undef, loc=>'vertical'},
 	
 	{ label=>" Disable all monitors: ", param_name=>'NO_MONITOR', type=>"Check-box", default_val=>'1\'b1', content=>1, info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}, 
 	{ label=>" Enable Pckt monitor: ", param_name=>'PCKT_MONITOR', type=>"Check-box", default_val=>'1\'b0', content=>1, info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'},     
	);   

my $simm = $self->object_get_attribute('CTRL','SIM') // " ";
my $ovs = { label=>" MPI oversubscribe", param_name=>'MPI_OVERSUB', type=>"Check-box", default_val=>'1\'b0', content=>1, info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'};    

	
 push (@info, $ovs) if($simm eq "Metro-MPI");

	my ($row,$col)=(0,0);

	foreach my $d (@info) {
    	my $wiget;        
      	($row,$col,$wiget)=add_param_widget  ($self, $d->{label}, $d->{param_name}, $d->{default_val}, $d->{type}, $d->{content}, $d->{info}, $table,$row,$col,1, $d->{param_parent}, $d->{ref_delay}, $d->{new_status}, $d->{loc});
        $col=0 if ($d->{loc} eq 'vertical');
		my $hbm_lat = $self->object_get_attribute('CTRL','HBM_LAT');
		if($d->{param_name} eq'HBM_LAT' && $hbm_lat eq 'Fix-lat'){
 			my $lats = (
				{ label=>" latency_clk: ", param_name=>'LAT_CLK',type=>"Spin-button", default_val=>0, content=>"0,1000,1",info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}
			);
			($row,$col,$wiget)=add_param_widget  ($self, $lats->{label}, $lats->{param_name}, $lats->{default_val}, $lats->{type}, $lats->{content}, $lats->{info}, $table,$row,$col,1, $lats->{param_parent}, $lats->{ref_delay}, $lats->{new_status}, $lats->{loc});
        	$col=0 if ($lats->{loc} eq 'vertical');
		}
		elsif($d->{param_name} eq 'HBM_LAT' && $hbm_lat eq 'Realistic'){
			#search for curves srcs in $PITON_ROOT/piton/tools/pli/Realistic_lat/curves_src
			
			opendir(my $dh, "$piton_root/piton/$lat_curve_dir") or die "can't opendir $piton_root/piton/$lat_curve_dir: $!";
			my @dirs = grep {-d "$piton_root/piton/$lat_curve_dir" && ! /^\.{1,2}$/} readdir($dh);
			close($dh);
			my $dirs = join (',',@dirs);
			my $lats = (
				{ label=>" latency_curve: ", param_name=>'LAT_CLK',type=>"Combo-box", default_val=>$dirs[0], content=>"$dirs",info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}
			);
			($row,$col,$wiget)=add_param_widget  ($self, $lats->{label}, $lats->{param_name}, $lats->{default_val}, $lats->{type}, $lats->{content}, $lats->{info}, $table,$row,$col,1, $lats->{param_parent}, $lats->{ref_delay}, $lats->{new_status}, $lats->{loc});
        	$col=0 if ($lats->{loc} eq 'vertical');
		}
	}		
	
	#@apps

	my $setting =def_image_button('icons/setting.png');
	$setting-> signal_connect("clicked" => sub{ 
		if($target ne "Local"){
		    run_server_setting();
		} else {
		    run_local_setting();		
		}
	}	);


	my $pronoc;
	$row=0;$col=6;
	($row,$col)=add_param_widget  ($self, "Target-dev", "TARGET", 'Local', "Combo-box", "Local,Remote", undef, $table,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	$table->attach ($setting, $col, $col+1, 0,1,'fill','shrink',2,2);
	
	$col=6;$row++;
	
	($row,$col,$pronoc)=add_param_widget  ($self, "ProNoC", "PRONOC", 'Disabled', "Combo-box", "Disabled,Enabled", undef, $table,$row,$col,1, 'CTRL', 0, 'ref_app', 'vertical');

	$col=6;$row++;
	
	my $pr = $self->object_get_attribute('CTRL',"PRONOC");
	if ($pr eq  'Enabled'){
		my $noc_table=def_table(1,1);
		my$r=noc_config($self,$noc_table);
		$table->attach ($noc_table, $col, $col+4, $row,$row+$r,'fill','shrink',2,2); $row+=$r;
	}
	my $scrolled_win = add_widget_to_scrolled_win($table,gen_scr_win_with_adjst($self,'bulid_scwin'));
	return $scrolled_win;
}

sub gen_mem_tab {
	my $table=def_table(1,1);
	
	my @info = (	
    { label=>" HBM RD latency: ", param_name=>'HBM_LAT', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Fix-lat,Realistic", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> 'ref_ctrl', loc=>'vertical'},    
	{ label=>" Multiple memory controllers ", param_name=>'MULTIMC', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Custom,ACME", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref', loc=>'vertical'},
 	); 

	my ($row,$col)=(0,0);

	foreach my $d (@info) {
    	my $wiget;        
      	($row,$col,$wiget)=add_param_widget  ($self, $d->{label}, $d->{param_name}, $d->{default_val}, $d->{type}, $d->{content}, $d->{info}, $table,$row,$col,1, $d->{param_parent}, $d->{ref_delay}, $d->{new_status}, $d->{loc});
        $col=0 if ($d->{loc} eq 'vertical');
		my $hbm_lat = $self->object_get_attribute('CTRL','HBM_LAT');
		if($d->{param_name} eq'HBM_LAT' && $hbm_lat eq 'Fix-lat'){
 			my $lats = (
				{ label=>" latency_clk: ", param_name=>'LAT_CLK',type=>"Spin-button", default_val=>0, content=>"0,1000,1",info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}
			);
			($row,$col,$wiget)=add_param_widget  ($self, $lats->{label}, $lats->{param_name}, $lats->{default_val}, $lats->{type}, $lats->{content}, $lats->{info}, $table,$row,$col,1, $lats->{param_parent}, $lats->{ref_delay}, $lats->{new_status}, $lats->{loc});
        	$col=0 if ($lats->{loc} eq 'vertical');
		}
		elsif($d->{param_name} eq 'HBM_LAT' && $hbm_lat eq 'Realistic'){
			#search for curves srcs in $PITON_ROOT/piton/tools/pli/Realistic_lat/curves_src
			
			opendir(my $dh, "$piton_root/piton/$lat_curve_dir") or die "can't opendir $piton_root/piton/$lat_curve_dir: $!";
			my @dirs = grep {-d "$piton_root/piton/$lat_curve_dir" && ! /^\.{1,2}$/} readdir($dh);
			close($dh);
			my $dirs = join (',',@dirs);
			my $lats = (
				{ label=>" latency_curve: ", param_name=>'LAT_CLK',type=>"Combo-box", default_val=>$dirs[0], content=>"$dirs",info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'}
			);
			($row,$col,$wiget)=add_param_widget  ($self, $lats->{label}, $lats->{param_name}, $lats->{default_val}, $lats->{type}, $lats->{content}, $lats->{info}, $table,$row,$col,1, $lats->{param_parent}, $lats->{ref_delay}, $lats->{new_status}, $lats->{loc});
        	$col=0 if ($lats->{loc} eq 'vertical');
		}
	}


	
	my $mmc =$self->object_get_attribute('CTRL','MULTIMC');
	if($mmc eq 'Custom'){
		$table->attach  ( gen_label_in_left(" Where do you want to connect MCs?") , 0, 1,  $row,$row+1,'fill','shrink',2,2); $row++;

		my $map = gen_mc_map ();
		$table->attach  ($map , 0, 4,  $row,$row+1,'fill','shrink',2,2);
	}
	
	#@apps

	
	my $scrolled_win = add_widget_to_scrolled_win($table,gen_scr_win_with_adjst($self,'bulid_scwin'));
	return $scrolled_win;
}

sub two_state_key {
	my ($name,$st1,$st2)=@_;
	my $st=$self->object_get_attribute("CTRL","$name");
	if(! defined $st){
		$st = $st1;
		$self->object_add_attribute("CTRL","$name",$st );
	}
	my $sw= ($st eq $st1)? def_colored_button($st1,17): def_colored_button($st2,4);
    $sw -> signal_connect("clicked" => sub{ 
			my $st=$self->object_get_attribute("CTRL","$name");	
			$st = $st1 if(! defined $st);		
			my $new = ($st eq $st1)? $st2 : $st1;			
			$self->object_add_attribute("CTRL","$name",$new);	
			set_gui_status($self,"ref",1);		
	});
	return $sw;
}

sub add_action_key {
	my($map_name,$table,$col,$row,$label,$action,$color)=@_;

	my $set= def_colored_button("$label",$color);
	$table->attach($set, $col, $col+1,$row,$row+1,'shrink','shrink',2,2); 
	$set -> signal_connect("clicked" => sub{ 
		$self->object_add_attribute('CTRL',"${map_name}_action",$action );	
		set_gui_status($self,"ref",1);		
	});
}

sub gen_tile_map_gui {
	my $map_name=shift;
	#get x, y dimention sizes
	my $x=$self->object_get_attribute('CTRL','X');
	my $y=$self->object_get_attribute('CTRL','Y');
	my $table1=def_table(1,1);
	my $table2=def_table(1,1);
	my $table=def_table(1,1);
	my ($row,$col)=(0,0);

	my $eq = def_table(1,8,TRUE);	
	my $label = gen_label_help("You can define an equation using x \& y variables 
	to define which tiles should have be configured with $map_name.
	\t Eg: (x\%2==1) \&\& (y\%2==0) : set tiles loacted in odd rows and even columns 
	\t Eg: 1                        : set all tiles
	\t Eg: x\%3!=0                  : set two tiles and then unset one and so on 
	","Equation:   ");

	my $entry =  gen_entry_object ($self, 'CTRL','${map_name}_Equation');
		
	my $open= def_image_button("icons/enter.png",undef,TRUE);
	$eq->attach ($label,0,2,  $row, $row+1,'fill','fill',2,2);
	$eq->attach_defaults ($entry,2, 9,  $row, $row+1);
	$eq->attach ($open,9, 10,  $row, $row+1,'fill','shrink',2,2);
	$table1->attach ($eq,0, 20,  $row, $row+1,'expand','fill',2,2);
	add_action_key(${map_name},$table1,20,$row,'Unset All',"Unset_Row_All",17);$row++;	
	
	$open->signal_connect("clicked" => sub {
		my @set_tiles;
		for (my $i=0; $i<$x; $i++){				
			for (my $j=0; $j<$y; $j++){
			my $string = $entry->get_text();	
			$string =~ s/x/$i/; # Replace 'x' with '$i'
			$string =~ s/y/$j/; # Replace ',' with '$j'
			my $r;
			my $n1 = eval $string;
			if(!defined $n1){
				 message_dialog("Can not evaluate $string: $r",'error');
				 return;
			}
				my $id= $j*$x+$i;
				push (@set_tiles,$id) if ($n1>0);
			}}	
			$self->object_add_attribute('CTRL',"${map_name}_set_array",\@set_tiles );
			set_gui_status($self,"ref",1);		

				
	});
	

	$row++;

	
	
	my @array1=('Row','Column');
	my @array2=('Set','Unset');
	foreach my $a1 (@array1){
		foreach my $a2 (@array2){			
			$col=0; $row++;
			my $color=($a2 eq "Set")? 4 : 17;
			my $size= ($a1 eq "Row")? $y : $x;
			#$table1->attach (gen_label_in_center("$a2 $a1#")  ,  $col, $col+1,$row,$row+1,'shrink','shrink',2,2);$col++;
			#add_action_key(${map_name},$table1,$col,$row,'All',"${a2}_${a1}_All",$color);$col++;
			#add_action_key(${map_name},$table1,$col,$row,'Odd',"${a2}_${a1}_Odd",$color);$col++;
			#add_action_key(${map_name},$table1,$col,$row,'Even',"${a2}_${a1}_Even",$color);$col++;
				
			for (my $i=0; $i<$size; $i++){
			#	add_action_key(${map_name},$table1,$col,$row,"$i","${a2}_${a1}_$i",$color);$col++;
			}
	
		}
	}



	my $action = $self->object_get_attribute('CTRL',"${map_name}_action");
	$action="nothing" if (!defined $action);
	my $ref = $self->object_get_attribute('CTRL',"${map_name}_set_array");
	for (my $i=0; $i<$x; $i++){				
		for (my $j=0; $j<$y; $j++){
			my $id= $j*$x+$i;
			my $f=$self->object_get_attribute('CTRL',"${map_name}_$id" );	
			$f="0" if (!defined $f);	
			my $set = 0;
			if (defined $ref){
				my @set_tiles =@{$ref};
					
				$set = check_scolar_exist_in_array($id,\@set_tiles) if(scalar @set_tiles);
			}

			my $en= $set == 1 ||
				($action eq "Set_Row_All") || 
				($action eq "Set_Column_All") || 
				($action eq "Set_Row_Odd" && $j%2==1)||
				($action eq "Set_Row_Even" && $j%2==0)||
				($action eq "Set_Column_Odd" && $i%2==1)||
				($action eq "Set_Column_Even" && $i%2==0)||
				($action eq "Set_Row_$j")||
				($action eq "Set_Column_$i")
				 ?  "1":
				
				($action eq "Unset_Row_All") || 
				($action eq "Unset_Column_All") || 
				($action eq "Unset_Row_Odd" && $j%2==1)||
				($action eq "Unset_Row_Even" && $j%2==0)||
				($action eq "Unset_Column_Odd" && $i%2==1)||
				($action eq "Unset_Column_Even" && $i%2==0)||
				($action eq "Unset_Row_$j")||
				($action eq "Unset_Column_$i")
				? "0": 	"$f";	
			$self->object_add_attribute('CTRL',"${map_name}_$id", $en);					
			add_param_widget  ($self, $id,"${map_name}_$id" , $en, "Check-box", 1, undef, $table2,($j+1)*4,($i+1)*4,1, 'CTRL', undef, undef, undef);
		}
	}
	$table->attach ($table1 ,  0, 1,0,1,'shrink','shrink',2,2); $row+=1;$col=0;
	$table->attach ($table2 ,  0, 1,1,2,'shrink','shrink',2,2); $row+=1;$col=0;
	
	$self->object_add_attribute('CTRL',"${map_name}_set_array",undef );

	return $table;
}

sub gen_tile_map_tab {
	my ($map_name)=@_;
	my $table=def_table(1,1);
	my ($row,$col)=(0,0);
	my $en_state=two_state_key("${map_name}_en","Disabled","Enabled");
	$table->attach (gen_label_in_center  ("Custom $map_name:") ,  $col, $col+1,$row,$row+1,'shrink','shrink',2,2); $col+=1;
	$table->attach ($en_state ,  $col, $col+1,$row,$row+1,'shrink','shrink',2,2); $row+=1;$col=0;


	my $en =$self->object_get_attribute('CTRL',"${map_name}_en");
	if($en eq 'Enabled'){
		$table->attach  ( gen_label_in_left(" Where do you want to map $map_name?") , 0, 1,  $row,$row+1,'fill','shrink',2,2); $row++;

		my $map = gen_tile_map_gui ($map_name);
		$table->attach  ($map , 0, 4,  $row,$row+1,'fill','shrink',2,2);
	}

	


	
	
	
	#@apps

	
	my $scrolled_win = add_widget_to_scrolled_win($table,gen_scr_win_with_adjst($self,'bulid_scwin'));
	return $scrolled_win;
}


sub gen_cache_tab{

	my $table=def_table(1,1);
	my ($row,$col)=(0,0);

	my @caches = ('l1i','l1d','l15','l2','hpdc');
	my @configurable = ('name','size','associativity','num_threads','req_width');
	foreach my $p (@configurable){
		$table->attach  ( gen_label_in_center("  $p  ") , $col, $col+1,  $row,$row+1,'fill','shrink',2,2);
		$col++;
	}
	$row++;
	$col=0;
	my $sizes="8192,16384,32768,65536,131072,262144,524288,1048576";
	foreach my $p (@caches){
		my $size_def = ($p eq 'l2' )? '65536' : ($p eq 'l1i' )? '16384' :'8192' ;
		my $w= gen_label_in_left(" $p ");
		$table->attach  ( $w , $col, $col+1,  $row,$row+1,'fill','shrink',2,2);$col++;
		
		if($p eq 'hpdc'){
		    $w=gen_combobox_object ($self,"CTRL","${p}_req_width","1,2,4,8",'1',undef,undef) ;
		    $table->attach  ( $w , 4, 5,  $row,$row+1,'fill','shrink',2,2);$col++;
		    $row++;
		    $col=0;
		    next;
        }
        	
		$w=gen_combobox_object ($self,"CTRL","${p}_size",$sizes,$size_def,undef,undef);
		$table->attach  ( $w , $col, $col+1,  $row,$row+1,'fill','shrink',2,2);$col++;
		$w=gen_combobox_object ($self,"CTRL","${p}_associativity","2,4,8,16,32,64,128",'4',undef,undef);
		$table->attach  ( $w , $col, $col+1,  $row,$row+1,'fill','shrink',2,2);$col++;
		
		if($p eq 'l15'){
		$w=gen_combobox_object ($self,"CTRL","${p}_num_threads","2,4,8,16,32,64,128,256",'2',undef,undef) ;
		$table->attach  ( $w , $col, $col+1,  $row,$row+1,'fill','shrink',2,2);$col++;
		}
		
		$row++;
		$col=0;
	}

	

	

	#{ label=>" L2_size: ", param_name=>'L2_SIZE', type=>"Combo-box", default_val=>'65536', content=>"65536,131072,262144,524288,1048576", info=>"L2 cache size per core", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},
	#{ label=>" L15_size: ", param_name=>'L15_SIZE', type=>"Combo-box", default_val=>'8192', content=>"8192,16384,32768,65536,131072", info=>"L1.5 cache size per core", param_parent=>'CTRL', ref_delay=> 1, new_status=>'ref_ctrl', loc=>'vertical'},




    #"config_l1i_size",
   # "config_l1i_associativity",
    #"config_l1d_size",
    #"config_l1d_associativity",
   # "config_l15_size",
   # "config_l15_associativity",
   # "config_l15_num_threads",
   # "config_l2_size",
   # "config_l2_associativity",

	my $scrolled_win = add_widget_to_scrolled_win($table,gen_scr_win_with_adjst($self,'bulid_scwin'));
	return $scrolled_win;

}





sub process_notebook_gen{
	my $notebook = gen_notebook();
	$notebook->set_tab_pos ('left');
	$notebook->set_scrollable(TRUE);
	my $page1 = gen_build_tab();
	$notebook->append_page ($page1,gen_label_with_mnemonic ("  _General  "));
	$notebook->append_page(gen_cache_tab( ),gen_label_with_mnemonic ("  _Caches  "));
   
	my $page2 = gen_mem_tab();
	$notebook->append_page ($page2,gen_label_with_mnemonic ("  _Memory  "));
	#$notebook->append_page(gen_tile_map_tab('l2_en' ),gen_label_with_mnemonic ("  _l2-map  "));
    #$notebook->append_page(gen_tile_map_tab('cpu_en'),gen_label_with_mnemonic ("  _cpu-map  "));

	my $page3 =gen_run_tab();
	$notebook->append_page ($page3,gen_label_with_mnemonic ("  _App  "));
	
	my $page4 =gen_results_tab();
	$notebook->append_page ($page4,gen_label_with_mnemonic ("  _Results  "));
	
	
	$notebook->show_all;
	my $page_num=$self->object_get_attribute ("process_notebook","currentpage");		
	$notebook->set_current_page ($page_num) if(defined $page_num);
	$notebook->signal_connect( 'switch-page'=> sub{			
		$self->object_add_attribute ("process_notebook","currentpage",$_[2]);	#save the new pagenumber
	});
		
	return $notebook;	
}		

sub gen_run_tab {
	my $table=def_table(1,1);
	my $target=$self->object_get_attribute('CTRL',"TARGET");
	$target = "Local "if(!defined $target);	
	my $sims = $target eq "Local" ? "Questasim,Verilator,Metro-MPI" : "Metro-MPI";
	
	my $app_server=$self->object_get_attribute('CTRL','APP-SERVER');
	$app_server = 'Local' if(!defined $app_server);
	
	my $core=$self->object_get_attribute('CTRL','TILE');
	my $list = ($core eq "ost1")? $sparc_app_list  :   $app_list;
	
	
	

	my @info = (
	 	
	{ label=>" custom App name: ", param_name=>'APP_CUSTOM', type=>"Entry", default_val=>undef, content=>undef, info=>undef, param_parent=>'CTRL', ref_delay=> undef, new_status=> undef, loc=>'vertical'},     
	{ label=>" App name: ", param_name=>'APP', type=>"Combo-box", default_val=>'hello_world_many.c', content=>$list, info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> 'ref', loc=>'vertical'}, 
	
	{ label=>" App location: ", param_name=>'APP-SERVER', type=>"Combo-box", default_val=>'Local', content=>"Local", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> 'ref', loc=>'vertical'},	
	);
	
	my $a=$self->object_get_attribute('CTRL','APP');
	my ($psize_def, $psize, $loc, $precomp)=get_app_info($a);
	if (defined $psize ){
	
	my $pow=1;
	my $content="1";
	while ($pow<10000000) {
		$pow*=2;
		$content.=",$pow";
	}
	
	push(@info,
	 { label=>" App Problem size: ", param_name=>'PROBLEM', type=>"Combo-box", default_val=>"$psize_def", content=>"$content", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	  { label=>" App Problem Type: ", param_name=>'DTYPE', type=>"Combo-box", default_val=>"double", content=>"double,float,int,char", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	 { label=>" Stack pointer KB: ", param_name=>'STK', type=>"Spin-button", default_val=>128, content=>"0,1024,1", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	{ label=>" Hex Dump Gen.: ", param_name=>'DUMP', type=>"Combo-box", default_val=>"Disabled", content=>"Disabled,Enabled", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	
	 );
	
	} 
	
	my $app_name=$self->object_get_attribute('CTRL','APP');
	$app_name="undef" if(!defined $app_name);

push(@info,
	 { label=>"Loop: ", param_name=>'LOOP', type=>"Spin-button", default_val=>1, content=>"1,1000,1", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	 { label=>"Pause: ", param_name=>'PAUSE', type=>"Spin-button", default_val=>0, content=>"0,1000,1", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
	 { label=>"RD ratio: ", param_name=>'RD_RATIO', type=>"Spin-button", default_val=>0, content=>"0,100,2", info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'}
 ) if($app_name eq 'mt-stream');	

		
my $metrics="Nothing,Cycles-instruction";
$metrics.=",hpm-metrics,hpm-custom-metrics" if($core eq 'sarg' || $core eq 'lox');


push(@info,
    {  label=>"Enable verification: ", param_name=>'VERIFY', type=>"Check-box", default_val=>'1\'b1', content=>1, info=>"If ticked then it verrify application results with expected results", param_parent=>'CTRL', ref_delay=> 1, new_status=> undef, loc=>'vertical'},
    
{  label=>"Printed Metrics: ", param_name=>'METRICS', type=>"Combo-box", default_val=>'Cycles-only', content=>$metrics, info=>undef, param_parent=>'CTRL', ref_delay=> 1, new_status=> 'ref', loc=>'vertical'},    
    
) if(defined $psize);

	
	my ($row,$col)=(0,0);
	foreach my $d (@info) {
       	 my $wiget;        
      	  ($row,$col,$wiget)=add_param_widget  ($self, $d->{label}, $d->{param_name}, $d->{default_val}, $d->{type}, $d->{content}, $d->{info}, $table,$row,$col,1, $d->{param_parent}, $d->{ref_delay}, $d->{new_status}, $d->{loc});
        $col=0 if ($d->{loc} eq 'vertical');
    

	}
	
	
	my $sel_metrics=$self->object_get_attribute('CTRL',"METRICS");
	
	if (defined $sel_metrics && $sel_metrics eq 'hpm-custom-metrics') {
	    my $seting =def_image_button('icons/setting.png',"select hpm metrics");
	    $seting-> signal_connect("clicked" => sub{ 
		    select_hpm_metrics();
	    }	);
	    $table->attach ($seting, $col, $col+1, $row,$row+1,'fill','shrink',2,2);$row++;

	}
	
	
	
	my $setting =def_image_button('icons/setting.png');
	$setting-> signal_connect("clicked" => sub{ 
		run_server_setting();
	}	);

	
	$row=0;$col=6;
	($row,$col)=add_param_widget  ($self, "Target-dev", "TARGET", 'Local', "Combo-box", "Local,Remote", undef, $table,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	$table->attach ($setting, $col, $col+1, 0,1,'fill','shrink',2,2)  if($target ne "Local");
	
	
	my $refresh =def_image_button('icons/refresh.png');
	$refresh-> signal_connect("clicked" => sub{ 
		get_list_of_models($target);
		set_gui_status($self,"ref",1);
	}	);

	if($target eq "Local"){
		get_list_of_models($target);
	}
	
	my $mlist = $self->object_get_attribute('CTRL',"MODEL_LIST");
	$mlist = " " if(!defined $mlist);
	$row=1;$col=6;
	($row,$col)=add_param_widget  ($self, "Select Model", "SEL_MODEL", undef, "Combo-box", $mlist, undef, $table,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	$table->attach ($refresh, $col, $col+1, $row,$row+1,'fill','shrink',2,2);# if($target ne "Local");
	
	
	
	
	return $table;
}
sub gen_results_tab {
	my $table=def_table(1,1);
	my $table1=def_table(1,1);
	my $target=$self->object_get_attribute('CTRL',"TARGET");
	$target = "Local "if(!defined $target);	
	my ($obj)= get_setting_object();
	my $supported_sims_server = $obj->object_get_attribute('SERVER',"SIMULATORS");
	my $sims = $target ne "Local" ? $supported_sims_server :"Questasim,Verilator,Metro-MPI";
	
	my ($row,$col)=(0,0);
	($row,$col)=add_param_widget  ($self, "Target-dev", "TARGET", 'Local', "Combo-box", "Local,Remote", undef, $table1,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	
	
	my $refresh =def_image_button('icons/refresh.png');
	set_tip($refresh,"Click to read the list of generated model on the server");
	$refresh-> signal_connect("clicked" => sub{ 	    
		get_list_of_models($target);
		set_gui_status($self,"ref",1);
	}	);
    

	if($target eq "Local"){
		get_list_of_models($target);
	}
	
	my $mlist = $self->object_get_attribute('CTRL',"MODEL_LIST");
	$mlist = " " if(!defined $mlist);
	$row=0;$col=6;
	my $model_name_def=$self->object_get_attribute('CTRL',"SEL_MODEL");
	($row,$col)=add_param_widget  ($self, "Select Model", "SEL_MODEL_RESULT", $model_name_def, "Combo-box", $mlist, undef, $table1,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	$table1->attach ($refresh, $col, $col+1, $row,$row+1,'fill','shrink',2,2)  if($target ne "Local");
	$col++ if($target ne "Local");
	
	$row++;$col=0;
	#my $sim_def =$self->object_get_attribute('CTRL','SIM');
	#($row,$col)=add_param_widget  ($self, " Simulator:", "SIM_RESULT", $sim_def, "Combo-box", $sims, undef, $table1,$row,$col,1, 'CTRL', 0, 'ref_app', 'horizontal');
	
	my $ref = def_image_button("icons/upload.png",'   Read Results');

    
    $col=6;
    $table1->attach( $ref,$col, $col+6, $row,$row+1,'fill','shrink',2,2); 
	
	
	my $notebook = gen_notebook();
	$notebook->set_tab_pos ('top');
	$notebook->set_scrollable(TRUE);
	
	
	
	my $page1 = read_file_tab($target,$ref,"fake_uart.log");
	my $page2 = read_file_tab($target,$ref,"sims.log");
	my $page3 = read_file_tab($target,$ref,"sims.log","Error");
	my $page4 = read_file_tab($target,$ref,"transcript");
	my $page5 = pckt_decoder  ();
	
	$notebook->append_page ($page1,gen_label_with_mnemonic ("  fake uart  "));
	$notebook->append_page ($page2,gen_label_with_mnemonic ("  sims.log  "));
	$notebook->append_page ($page3,gen_label_with_mnemonic (" Errors sims.log  "));
	$notebook->append_page ($page4,gen_label_with_mnemonic (" Transcript "));
	$notebook->append_page ($page5,gen_label_with_mnemonic (" packet decoder "));
	#my $scrolled_win= add_widget_to_scrolled_win($notebook);
	my $table2=def_table(1,1);
	$table2->attach_defaults ($notebook, 0, 1, 0,1);
	$row++;
	$table->attach ($table1, 0,1, 0,1,'fill','shrink',2,2);
	$table->attach_defaults ($table2, 0,1, 1,2);
		
	return $table;
}

sub pckt_decoder {
    my ($box1,$txv1)=create_txview(); 
    my ($box2,$txv2)=create_txview(); 
    my $buffer = $txv1->get_buffer();
	show_info($txv1,"insert packet content here: E.g\n
# ------------------------------------
# [<-] N1 out id:14
#       #Pck:                   2
#       #pck size:      3
#       Data          0:  0000100000838140
#       Data          1:  00fff10100080c00
#       Data          2:  0000100800000000
# ------------------------------------
	
	" );

    # Connect to the "insert-text" signal of the text buffer
    $buffer->signal_connect(insert_text => sub{
	    show_info($txv2,"decoded packet:  \n" );		
		my $start_iter = $buffer->get_start_iter();
    	my $end_iter = $buffer->get_end_iter();
    	my $text = $buffer->get_text($start_iter, $end_iter, 1);  # 1 indicates to include hidden characters
        
		my @lines = split /\n/, $text;
		my $noc;
    	foreach my $line (@lines) {
			if ($line =~ /\[<-\] N(\d+)/) {
				$noc = $1;
				add_info($txv2,"noc=$noc \n" );		
			}
			if ($line =~ /Data\s+(\d+):\s+([0-9a-fA-F]+)/) {
				my $num = $1;
				my $hexvalue = $2;
			    my $dec = decode_pck ($noc,$num, $hexvalue);
				add_info($txv2,"$dec \n" );			
			}
    	}		
	}
	);
    my $table=def_table(1,1);
    $table->attach_defaults ($box1, 0,1, 0,1);
    $table->attach_defaults ($box2, 1,2, 0,1);
    return $table;
}

sub filter_file_content {
    my ($txt,$filter)=@_;
    my $out="";
    # Split the content into lines
    my @lines = split(/\n/, $txt);

    # Loop through each line and check for 'Error'
    for my $i (0 .. $#lines) {
        if ($lines[$i] =~ /$filter/) {
            # Print the line before, the error line, and the line after (if they exist)
            $out.= "$lines[$i-1]\n" if $i > 0;     # Line before the Error
            $out.= "$lines[$i]\n";                 # Error line
            $out.= "$lines[$i+1]\n" if $i < $#lines; # Line after the Error
            $out.= "\n";  # Separate different blocks            
        }
    }
    return $out;
}


sub read_file_tab {
    my ($target,$ref,$file,$filter)=@_;
    my ($box,$txv)=create_txview();  
    

    $ref->signal_connect("clicked"=> sub{
        my $simulator = $self->object_get_attribute('CTRL','SIM');
        my $model_name=$self->object_get_attribute('CTRL',"SEL_MODEL_RESULT");
        if ($target eq 'Local') {
          my $path=  "$piton_root/build/$simulator/$model_name/$file";
          if (-f 	$path ){
            my $txt = read_file_cntent($path,"");
            $txt = filter_file_content( $txt,$filter)   if(defined $filter);               
            show_info($txv,"File: $path\n         ************************************\n$txt" );
          }else {
            show_info($txv,"Error $path does not exist!" );
          }
          return;
        }
       
       
       my ($uname) =get_server_id();
	   my $repo = abs_path("$piton_root");
	   my $top= basename($repo);
	   my $ssh = get_ssh_cmd();
	   if(!defined $uname){	
	    show_info($txv,"Error: username in server is unkown!"); 
	    return;
	   }
	   my $cmd ="$ssh cat $top/build/$simulator/$model_name/$file";
	   my ($txt)=run_cmd_in_back_ground_get_stdout($cmd );
	   $txt = filter_file_content( $txt,$filter)   if(defined $filter);    
       show_info($txv,"$txt" );
       
    });    

    return $box;

}

sub get_list_of_models {
	my $target=shift;
	my $sim =$self->object_get_attribute('CTRL','SIM');
	
	return if (!defined $sim && $target eq 'Local' );
	
	my $list;
	my @files;
	 
	if($target eq 'Local'){
		#get list of all subdir has param file
		my $path="$piton_root/build/$sim";
		@files = glob "$path/*/param";
	}else{
	
	    if(!defined $sim){
	        message_dialog("simulator is undefined. Please select the simulator first\n",'error');
	        return;	
	    }
		my ($uname) =get_server_id();
		return if(!defined $uname);
		my $repo = abs_path("$piton_root");
		my $top= basename($repo);	
		my $ssh = get_ssh_cmd();
		my $bash = "$ssh find $top/build/$sim/ -name \"param\""; 
		my $out = run_cmd_message_dialog_errors($bash);
		@files =split("\n",$out);
	
	}	
		
	foreach my $f (@files){
		my @subdir =split('/',$f);
		my $fd = $subdir[-2];
		
		if(defined $list) {
		 	$list.=",$fd" if(defined $fd);
		 }else{
		 	$list="$fd" if(defined $fd);
		 }
		
	}
	
	$self->object_add_attribute('CTRL',"MODEL_LIST",$list);
	
	
}


my $window = def_popwin_size(85,85,"gui",'percent');
$window->signal_connect (delete_event => sub { Gtk3->main_quit });

my $mtable = def_table(10, 1, FALSE);
my $table = process_notebook_gen();
my $ctrl  = get_ctrl_btm();
$mtable->attach_defaults($table,0,4,0,19);
$mtable->attach_defaults($ctrl,0,4,20,21);

my ($box,$txv)=create_txview();

my $refbox=gen_spin_object ($self,'CTRL',"REFRESH","1,1000,1", 2);

my $ref = def_image_button("icons/refresh.png");
my $label  = gen_label_in_center("Refresh x30s:");
my $ll = def_pack_hbox(0,0,$ref,$label);

$ref->signal_connect("clicked"=> sub{
    $self->object_add_attribute('CTRL',"ACTIVE",'enabled');    
    $ref_squeue_now=1;
});

my $cancel = def_image_button("icons/cancel.png","Kill all jobs");

$cancel->signal_connect("clicked"=> sub{
    my ($uname,$run_queue,$build_queue) =get_server_id();
    my $ssh = get_ssh_cmd(); 
   
    if ($run_queue ne 'UNDEF' or $build_queue ne 'UNDEF') {
        my $bash =  "$ssh \"squeue --me -h -o %i | xargs scancel \"";
        run_cmd_message_dialog_errors($bash);	
        $ref_squeue_now=1;	
        return;
    }
    
    $self->object_add_attribute('CTRL',"KILL_PS", 1);   
    $ref_squeue_now=1;    
    
});

$mtable->attach( $ll,4,5,0,1,'shrink','shrink',2,2); 
$mtable->attach($refbox,5,6,0,1,'shrink','shrink',2,2); 
$mtable->attach($cancel,4,6,1,2,'shrink','shrink',2,2); 
$mtable->attach($box,4,6,2,20,'fill','fill',2,2); 


$window->add ($mtable);
$window->show_all();





my $counter=0;
Glib::Timeout->add (100, sub{ 
	
	my ($state,$timeout)= get_gui_status($self);
	if ($timeout>0){
		$timeout--;
		set_gui_status($self,$state,$timeout);
		
	}
	
	
	elsif( $state ne "ideal" ){
		
		$table->destroy;
		$ctrl->destroy;
		$table = process_notebook_gen();
		$ctrl  = get_ctrl_btm();
		$mtable->attach_defaults($table,0,4,0,19);
		$mtable->attach_defaults($ctrl,0,4,20,21);
		$window->show_all;
		set_gui_status($self,"ideal",0);
	}
	#get active job status from the remote server
	my $en = $self->object_get_attribute('CTRL',"ACTIVE");
	$en="disable" if(!defined $en);
	my $ref_at = $self->object_get_attribute('CTRL',"REFRESH");
	
	if($en eq 'enabled'){
		$counter++;
		if($counter >=300*$ref_at || $ref_squeue_now == 1){
			$counter=0;
			$ref_squeue_now=0;
			my $target=$self->object_get_attribute('CTRL',"TARGET");
			my ($uname,$run_queue,$build_queue) =get_server_id();
			die "Error: username in server is unkown!" if(!defined $uname);
									
			
			my $ssh = get_ssh_cmd();
			my $bash= ($run_queue eq 'UNDEF' or $build_queue eq 'UNDEF') ? "$ssh ps -u `$ssh whoami`" : "$ssh squeue -u `$ssh whoami`";
			my $id = run_cmd_message_dialog_errors($bash);			
			
			my @list = split("\n",$id);
			if(!defined $list[1]){
				show_info($txv,"No active request in system queue!");
			 	$self->object_add_attribute('CTRL',"ACTIVE",'disable');
			}
			else{
				my ($out)=($run_queue eq 'UNDEF' or $build_queue eq 'UNDEF')? ps_active_job($id): squeu_jobs_summary($id);
				show_info($txv,"Slurm:\n$out" );
			}
		}
	}
	
	 $en =$self->object_get_attribute('CTRL',"KILL_PS");  
	 $en=0 if(!defined $en); 
	 if($en ){
        my $ssh = get_ssh_cmd();
	    my $bash=  "$ssh ps -u `$ssh whoami`";
 	    my $id = run_cmd_message_dialog_errors($bash);	
 	    my ($output,@pids) = ps_active_job($id);				   
	    return unless( scalar @pids);
	    my $kill= "$ssh kill " . join(' ', @pids);
	    run_cmd_message_dialog_errors($kill);
	    $self->object_add_attribute('CTRL',"KILL_PS",0);  
	    $self->object_add_attribute('CTRL',"ACTIVE",'enable');	
	    $ref_squeue_now=1;
	}
	
	
	return TRUE;
}
);

sub get_app_info {
	my ($name,$app_server)=@_;
	return (undef,undef,undef,undef,undef) if(!defined $name);
	$app_server = 'Local' if(!defined $app_server);
	
	my @list =  @apps;
	my $app_root= "piton/tools/metro_mpi/benchmark/app-scalar/";    
	foreach my $d (@list) {
    		if($d->{name} eq $name){
    			my $psize_def = $d->{psize_def};
    			my $psize     = $d->{psize};
    			my $loc       = ($app_server eq 'Local')? "$app_root/$name" : $d->{loc}  ;
    			my $precomp   = $d->{precomp};    			
    			return ($psize_def ,$psize  , $loc,  $precomp );  
    		}
	}
	return (undef,undef,undef,undef,undef);
}


sub run_bash {
	my $mode=shift;
	my $model_obj;
	
	my $a=$self->object_get_attribute('CTRL','APP');
	my $problem=$self->object_get_attribute('CTRL','PROBLEM');
	my $dtype =$self->object_get_attribute('CTRL','DTYPE');
	my $sim =$self->object_get_attribute('CTRL','SIM');
	my $target=$self->object_get_attribute('CTRL',"TARGET");
	my $stk=$self->object_get_attribute('CTRL','STK');
	$stk = (defined $stk)? $stk*=1024 : "";
	my $dump=$self->object_get_attribute('CTRL','DUMP');
    $dump = (defined $dump && $dump eq "Enabled")? "-x" : "";
    $problem = 0 if (!defined $problem);
	my $verrify=$self->object_get_attribute('CTRL',"VERIFY");
	my $metrics=$self->object_get_attribute('CTRL',"METRICS");

	
		
	if(!defined $sim){
	    message_dialog("simulator is undefined. Please select the simulator\n",'error');
	    return;	
	}	

	my $local_init = ($target eq 'Local' ) ? get_local_init_setting() : ""; 

		
	if($mode eq  'run') {
		#get model name
		my $model_path;
		my $model_name=$self->object_get_attribute('CTRL',"SEL_MODEL");
		my $path;
		if($target eq 'Local'){
			$path="$piton_root/build/$sim/$model_name/param";
			
		}else{
			my ($uname) =get_server_id();
			my $repo = abs_path("$piton_root");
			my $top= basename($repo);	
			die "Error: username in server is unkown!" if(!defined $uname);
			unlink ("$piton_root/build/tmp");
			my $bash =" scp ${uname}:$top/build/$sim/$model_name/param $piton_root/build/tmp";
			my $id = run_cmd_message_dialog_errors($bash);
			$path="$piton_root/build/tmp";
		}
		my ($r,$err);
		($model_obj,$r,$err) = regen_object($path);
		if ($r){        
		    die "**Error reading  $path file: $err\n";
		} 
					
		
	}else{
		$model_obj=$self;
	
	}
	
	
	
	
	my $x=$model_obj->object_get_attribute('CTRL','X');
	my $y=$model_obj->object_get_attribute('CTRL','Y');
	my $core=$model_obj->object_get_attribute('CTRL','TILE');
	my $l2_size=$model_obj->object_get_attribute('CTRL','L2_size');
	my $l15_size=$model_obj->object_get_attribute('CTRL','L15_size');

    my $caches = ""; 
	
	my @caches = ('l1i','l1d','l15','l2');
	my @configurable = ('size','associativity','num_threads');
	foreach my $n (@caches){
		my $size_def = ($n eq 'l2' )? '65536' : ($n eq 'l1i' )? '16384' :'8192' ;
		my $val = $model_obj->object_get_attribute('CTRL',"${n}_size");
		$caches .= "-config_${n}_size=$val " if (defined $val && $val ne $size_def);
		$val = $model_obj->object_get_attribute('CTRL',"${n}_associativity");
		$caches .= "-config_${n}_associativity=$val " if (defined $val && $val ne '4');
		$val = $model_obj->object_get_attribute('CTRL',"${n}_num_threads");
		$caches .= "-${n}_num_threads=$val " if (defined $val && $val ne '2');

	}
	my $val = $model_obj->object_get_attribute('CTRL',"hpdc_req_width");
   # $caches .= "-config_hpdc_req_words=$val " if (defined $val );

	
	my $noc1_width = $model_obj->object_get_attribute('CTRL','NOC1_WIDTH');
	my $noc2_width = $model_obj->object_get_attribute('CTRL','NOC2_WIDTH');
	my $noc3_width = $model_obj->object_get_attribute('CTRL','NOC3_WIDTH');
	
	my $noc_width= "";	
	#$noc_width.="-noc1_width=$noc1_width " ;#unless ($noc1_width eq '64');
	#$noc_width.="-noc2_width=$noc2_width " ;#unless ($noc2_width eq '64');
	#$noc_width.="-noc3_width=$noc3_width " ;#unless ($noc3_width eq '64');
	
	
	my $mgui=$model_obj->object_get_attribute('CTRL','MANY_GUI');
	my $rtlt=$model_obj->object_get_attribute('CTRL','RTLTIM'); 
	my $w=$model_obj->object_get_attribute('CTRL','MODEL'); 
	my $lat= $model_obj->object_get_attribute('CTRL','HBM_LAT'); 
	my $lat_clks =$model_obj->object_get_attribute('CTRL','LAT_CLK');  
	
	
	
	$lat = 
		($lat eq 'Fix-lat')  ? "-l $lat_clks" : 
		($lat eq 'Realistic')? "-l $lat_curve_dir/$lat_clks" :
		"";
	
	my $mon_st = $model_obj->object_get_attribute('CTRL','NO_MONITOR');
	my $monitor = ($mon_st eq '1\'b1') ? "-d" : "";	
	
	
	my $pr = $self->object_get_attribute('CTRL',"PRONOC");
	my $pronoc_arg="";
	if ($pr eq  'Enabled'){	
		#check if we have custom NoC
		my $Topology=$model_obj->object_get_attribute('noc_param','Topology'); 
		if ($Topology eq 'Different') {
			my $TOPOLOGY = $model_obj->object_get_attribute('noc_param','TOPOLOGY');
			my $T1 = $model_obj->object_get_attribute('noc_param','T1');
			my $T2 = $model_obj->object_get_attribute('noc_param','T2');
			my $T3 = $model_obj->object_get_attribute('noc_param','T3');
			$pronoc_arg.="TOPOLOGY=$TOPOLOGY,T1=$T1,T2=$T2,T3=$T3,";
		}

		my $SMART_MAX = $model_obj->object_get_attribute('noc_param','SMART_MAX');
		my $ADD_PIPREG_AFTER_CROSSBAR = $model_obj->object_get_attribute('noc_param','ADD_PIPREG_AFTER_CROSSBAR');
		my $SWA_ARBITER_TYPE = $model_obj->object_get_attribute('noc_param','SWA_ARBITER_TYPE');
		my $WEIGHTw = $model_obj->object_get_attribute('noc_param','WEIGHTw');
		$pronoc_arg.="SMART_MAX=$SMART_MAX,ADD_PIPREG_AFTER_CROSSBAR=$ADD_PIPREG_AFTER_CROSSBAR,SWA_ARBITER_TYPE=$SWA_ARBITER_TYPE,WEIGHTw=$WEIGHTw";
					
	}	



	$w="build/$sim/$w";
	my $p=$model_obj->object_get_attribute('CTRL','PRONOC'); 
	my $mmc =$model_obj->object_get_attribute('CTRL','MULTIMC');
	#my $target=$model_obj->object_get_attribute('CTRL',"TARGET");
	
	$p = ($p eq "Enabled") ? "-p \"$pronoc_arg\" " : "";
	if(	$mmc eq "Custom"){
		my ($n,$l) = get_custom_locs($model_obj,"mc");
		$mmc = ($n >0) ? "-m \"$l\" ": "";
	
	}elsif($mmc eq "ACME"){
		$mmc ="-m \"acme\" "	
	}else{
		$mmc = "";	
	}
	
	#my $custom="";
	#foreach my $c ('l2_en','cpu_en'){
	#	my $custom_en =$model_obj->object_get_attribute('CTRL',"${c}_en");
	#	if ($custom_en eq 'Enabled'){
	#		my ($n,$l) = get_custom_locs($model_obj,"$c");
	#		$custom .= ($n >0) ? "-custom_$c=$l ": "-custom_$c=NONE";
	#	}
	#}

    my $t=$self->object_get_attribute('CTRL','MPI_OVERSUB') // "0";
	my $hpdc = ($model_obj->object_get_attribute('CTRL',"HPDC") eq 'Enabled') ? '-hpdc' : "" ;
	my $pck_mon =($model_obj->object_get_attribute('CTRL','PCKT_MONITOR') eq '1\'b1') ? '-config_rtl PITON_PCK_MON' : "";

    my $mpi_ovs=($t eq '1\'b1') ? '-o' : "";

	my $extra = "-e \"$caches $noc_width $hpdc $pck_mon\" ";
	
	$mgui = ($mgui eq 'Enabled')? "-g  " : "";
	
	
	my $app;
	my $compile;
	my $defines;
	
	if (defined $metrics) {
	    $defines="-DREPORT_OP_METRICS "  if($metrics eq "Cycles-instruction");
	    $defines="-DREPORT_HPM_METRICS " if($metrics eq "hpm-metrics");
	    $defines="-DREPORT_CUSTOM_HPM " if($metrics eq "hpm-custom-metrics");
	    $defines.="-DVERYFY_RESULT " if ($verrify eq '1\'b1');	   
	}
	$defines= (defined $defines)?  "-D \"$defines\"" : "";
	
	my ($psize_def ,$psize  , $loc, $precomp )=get_app_info($a,'Local');
	
	my $custom_app=$self->object_get_attribute('CTRL','APP_CUSTOM') // ""; 
	
	(my $trimmed =$custom_app ) =~ s/\s+//g;
	if(length($trimmed)>1){
	    $app="\" $custom_app \"";
	}
	elsif ($precomp){
		my $c = $x * $y;
		
		
		my $app_server=$self->object_get_attribute('CTRL','APP-SERVER');
		$app_server = 'Local' if(!defined $app_server);
		
		if($app_server eq 'Local'){
		
		my $app_loop=$self->object_get_attribute('CTRL','LOOP');
		my $app_pause=$self->object_get_attribute('CTRL','PAUSE');
	 	my $app_rd_ratio=$self->object_get_attribute('CTRL','RD_RATIO');
	 	my $e = ($a eq 'mt-stream') ? "-e \"--loop=$app_loop --pause=$app_pause --RD_ratio=$app_rd_ratio\" " : "";
	 	    my $sel_metrics1=$self->object_get_attribute('CTRL',"METRICS");
	
	        if (defined $sel_metrics1 && $sel_metrics1 eq 'hpm-custom-metrics') {
	 	        gen_hpm_h();
		    }
			my $gen = "$local_init cd ..; bash compile.sh $dump -c $core -p $problem  -m $c -a $a -d $dtype -S $stk $e $defines" ;		
			$compile= "cd $piton_root/$loc; rm bin/*; $gen; cd $current_dir";
		}
		
		
		
		#copy 
		if($target eq 'Local'){
			$app	="\" -precompiled  ${core}_${a}${c}_${problem}.riscv -asm_diag_path $piton_root/$loc/bin \"  ";
		}else{
			my ($uname) =get_server_id();
			my $repo = abs_path("$piton_root");
			my $top= basename($repo);
			my $ssh = get_ssh_cmd();
			die "Error: username in server is unkown!" if(!defined $uname);
			$compile .="; $ssh mkdir -p $top/$loc/bin; $ssh rm $top/$loc/bin/*; scp $piton_root/$loc/bin/*  ${uname}:$top/$loc/bin/";
			$app	="\" -precompiled   ${core}_$a${c}_${problem}.riscv -asm_diag_path ~/$top/$loc/bin \"  ";
		}
	}else{
		$app	=$a;
	}
	
	
	#add verilator root 
    my ($ver,$verilator_bin,$verilator_repo) = get_verilator_setting();
    my $verilator_init=($target eq 'Local' ) ? "export VERILATOR_ROOT=$verilator_bin; $local_init" : "";
   
	my $cmd;
	if($sim eq 'Metro-MPI') {		
		my $flag= ($mode eq 'run') ? "-r" : ($mode eq 'build') ? '-c -t -f' : '-c -f -t -r';
		$cmd="module load $MODULES;" if (defined $MODULES && $target ne "Local");
		$cmd.=" $verilator_init bash ./metro_mpi_run.sh -x $x -y $y -z $core -w $w $p $flag -a $app $mmc $mgui $lat $monitor $extra $mpi_ovs";
		$cmd.=$red_out if($target eq "Local");
     	}elsif($sim eq 'Questasim') {
     		my $flag= ($mode eq 'run') ? "-r" : ($mode eq 'build') ? '-b' : '-b -r';
		$cmd=" $local_init bash ./Questasim_run.sh -x $x -y $y -z $core -w $w $p $flag -a $app $mmc $mgui $lat $monitor $extra -t $rtlt"; 
		    	
     	}else{
     	my $flag= ($mode eq 'run') ? "-r" : ($mode eq 'build') ? '-b' : '-b -r';
		$cmd=" $verilator_init bash verialtor_run.sh -x $x -y $y -z $core -w $w $p $flag -a $app $mmc $mgui $lat $monitor $extra";     	
     	}
	
	if(defined $compile && $mode eq  'run' ){
	 	run_in_xterm($compile);
	 		
	}
	my $param_file="$piton_root/$w/param";
	system("mkdir -p $piton_root/$w");
	if($mode eq 'build'){
		#save setting
		open(FILE,  ">$param_file") || die "Can not open: $!";
		print FILE Data::Dumper->Dump([\%$model_obj],['glob']);
		close(FILE) || die "Error closing file: $!";
		set_gui_status($self,"ref",1);
	
	}
	
	if($target eq 'Local' ){
		 run_in_xterm($cmd);
		 
		 
		 
	}else{
		my ($n,$l) = get_custom_locs($model_obj,'mc');
		my $core_num=($mode eq  'run')? $x * $y +1 + $n :10 ;	
		my_server_run($cmd,$w,$mode,$param_file,$core_num);	
	}
	
	
	 
}

sub copy_repo_on_server_in_xterm{
    my $dir = dirname(__FILE__);
	my ($uname,$run_queue,$build_queue,$verilator,$sbatch,$core_per_node,$shell) =get_server_id();
    #create initial setting file on the server
    my $file="#!/bin/bash
        $sbatch
    ";
    save_file("$tmp1",$file);        
	my $bash = "scp $tmp1 ${uname}:initial_settup.sh; cd $dir; bash ./server_init.sh -u $uname -c -v $verilator  -m \"source initial_settup.sh\" ";
	run_in_xterm($bash);
}


sub run_in_xterm{
	my ($cmd,$timeout)=@_;
	$timeout= 10 if(!defined $timeout);
	my $bash= "xterm   -e bash -c \' $cmd; echo \"\n\nDone! This window will be closed after 10 second\n\n\";  sleep $timeout; wait; \'";
	print " $cmd \n";
	run_cmd_message_dialog_errors($bash);
}

my $job_id=0;



sub get_server_id {
    my $paths_file; 
	($obj_server,$paths_file)= get_setting_object();
	
	my $uname  = $obj_server->object_get_attribute('SERVER',"UNAME");
	my $run_queue = $obj_server->object_get_attribute('SERVER',"RUN_QUEUE") // "UNDEF";
	my $build_queue=$obj_server->object_get_attribute('SERVER',"BUILD_QUEUE") // "UNDEF";
	my $verilator =$obj_server->object_get_attribute('VERILATOR',"DEFAULT");
	my $core_per_node = $obj_server->object_get_attribute('SERVER','MAX_CORE_PER_NODE') //1;
	my $mpi_alloc_mode= $obj_server->object_get_attribute('SERVER','MPI_JOB_ALLOCATION_MODE') //1;

	my $shell = $obj_server->object_get_attribute('SERVER','SHELL') // "bash";
	
	my $sbatch;
	my @bash_sel; 
	my $r= $obj_server->object_get_attribute('SBATCHS_EN');
	@bash_sel = @{$r} if (defined $r);
    my $sbatch_ref =$obj_server->object_get_attribute('SBATCH');    
    if(defined $sbatch_ref){
	    my %hash = %{$sbatch_ref};
	    $sbatch="";
	    foreach my $p (sort  {$a <=> $b} (keys %hash)){
	     $sbatch.="$hash{$p} \n" if (grep { $_ == int($p) } @bash_sel );
	      
	    }	
    }
	
	return ($uname,$run_queue,$build_queue,$verilator,$sbatch,$core_per_node,$shell,$mpi_alloc_mode);
}


sub my_server_run {
	my ($cmd,$w,$mode,$param_file,$core_num)=@_;
	
	
	my ($uname,$run_queue,$build_queue,$verilator,$sbatch,$core_per_node,$shell,$mpi_alloc_mode) =get_server_id();
	return if(!defined $uname);
	
	my $queue = ($mode eq 'build') ? $build_queue : $run_queue;
	
	my $repo = abs_path("$piton_root");
	my $top= basename($repo);	
	
	my $file="#!/bin/bash
cd  ~/$top/piton/tools/metro_mpi
$cmd
";

	save_file("$tmp1",$file);
my $work="~/meep_openpiton/$w/script";
my $job=$w;
$job =~ s/\//-/g;
$job.=$job_id;
$job_id++;


my $node_num = int ($core_num/int($core_per_node));
$node_num++;

  my $sbatch_task_config =
       ($mpi_alloc_mode eq '1' ) ? 
"#SBATCH --cpus-per-task=$core_num
#SBATCH --ntasks=1
"  :
"
#SBATCH --cpus-per-task=1
#SBATCH --ntasks=$core_num
";
	
	$file="#!/bin/bash



#SBATCH --job-name=\"$job\"
#SBATCH --output=r.out
#SBATCH --error=r.err
#SBATCH --nodes=$node_num
#SBATCH --qos=$queue
#SBATCH --time=1:00:00

$sbatch_task_config
$sbatch

mkdir -p ./out;
time bash ./build.sh > ./out/${job}_log

";


    save_file("$tmp2",$file);
	
	my $ssh = get_ssh_cmd();
	#create run.sh file	
	my $bash="$ssh mkdir -p $top/$w/script; scp $tmp1 ${uname}:$top/$w/script/build.sh; scp $tmp2 ${uname}:$top/$w/script/Queue_build.sh;   $ssh chmod +x $top/$w/script/Queue_build.sh";
	$bash.=";  scp $param_file ${uname}:$top/$w/param"  if ($mode eq 'build');
	
	run_in_xterm($bash);
	
	$bash="$ssh \" cd $top/$w/script/; $shell Queue_build.sh\"";
	my $id = run_cmd_message_dialog_errors($bash) if($queue ne 'UNDEF');	
	add_active_job(0);
	run_in_xterm($bash) if($queue eq 'UNDEF');
	
	#$id=~ s/[^0-9]//g;

	unlink($param_file) unless($queue eq 'UNDEF');
	return;
}

sub noc_config{
    my ($mpsoc,$table,$txview)=@_;
    
    #title    
    my $row=0;
    my $title=gen_label_in_center("ProNoC Configuration");
    $table->attach ($title , 0, 4,  $row, $row+1,'expand','shrink',2,2); $row++;
	add_Hsep_to_table ($table,0,4,$row); $row++;
   
    my $label;
    my $param;
    my $default;
    my $type;
    my $content;
    my $info;
	my $coltmp=0;
    
    #ProNoC Topology
    $label= "ProNoC Topology";
    $param='Topology';
    $default='Same-as-OP';                                  
    $content='Same-as-OP,Different';
    $type='Combo-box';
    $info='Default: having same topology with OP. Custom: chan change NoC dimention and add concentaration';
    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,1,'noc_param',1);

	my $Topology=$mpsoc->object_get_attribute('noc_param','Topology'); 
   
    if ($Topology eq 'Different') {
		 ($row,$coltmp) =noc_topology_setting_gui($mpsoc,$table,$txview,$row,1);
		 
	}


    
    #SMART
    $label='Max Straight Bypass'; 
    $param='SMART_MAX';
    $default='0';
    $content="0,1,2,3,4,5,6,7,8,9";
    $type='Combo-box';
    $info="If Max Straight Bypass (SMART_MAX) is defined as n>0 then packets are allowed to bypass Maximum of n routers in Straight direction in single cycle."; 
    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,1,'noc_param',undef);
   
    #pipeline reg    
    $label="Add pipeline reg after crossbar";    
    $param="ADD_PIPREG_AFTER_CROSSBAR";
    $type='Combo-box';
    $content=1;
    $default=0;
	$content="0,1";
    $info="If is enabled it adds a pipeline register at the output port of the router.";
    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,1,'noc_param');
        
    #Arbiter type
    $label='SW allocator arbitration type'; 
    $param='SWA_ARBITER_TYPE';
    $default='"RRA"';
    $content='"RRA","WRRA"'; #,"WRRA_CLASSIC"';
    $type='Combo-box';
    $info="Switch allocator arbiter type: 
    RRA: Round robin arbiter. Only local fairness in a router. 
    WRRA: Weighted round robin arbiter. Results in global fairness in the NoC. 
          Switch allocation requests are grated according to their weight which increases due to contention"; 
    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,1,'noc_param',1);
     
    
    my $arbiter=$mpsoc->object_get_attribute('noc_param',"SWA_ARBITER_TYPE");
    my $wrra_show = ($arbiter ne  '"RRA"'  )? 1 : 0;
    # weight width
    $label='Weight width';
    $param='WEIGHTw';
    $default='4';
    $content='2,7,1';
    $info= 'Maximum weight width';
    $type= 'Spin-button';  
    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,$wrra_show,'noc_param',undef);  
        
    return $row;
}

sub noc_topology_setting_gui {
	my ($mpsoc,$table,$txview,$row,$show_noc)=@_;
	my $coltmp=0;
	#  topology
	my  $label='Topology';
	my  $param='TOPOLOGY';
	my  $default='"FMESH"';
	my  $content='"MESH","FMESH","TORUS","RING","LINE","FATTREE","TREE","STAR","CUSTOM"';
	my  $type='Combo-box';
	my  $info="NoC topology"; 
	($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,$show_noc,'noc_param',1);
            
    my $topology=$mpsoc->object_get_attribute('noc_param','TOPOLOGY');

	if($topology ne '"CUSTOM"' ){
    #topology T1 parameter
	    $label= 
	    	($topology eq '"FATTREE"' || $topology eq '"TREE"')? 'K' :
	     	($topology eq '"STAR"')? "Total Endpoint number" : 'Routers per row';
	    $param= 'T1';
		$default= '2';
	    $content=
	    ($topology eq '"MESH"'  || $topology eq '"TORUS"') ? '2,16,1':
	    ($topology eq '"FMESH"')? '1,16,1':
		($topology eq '"FATTREE"' || $topology eq '"TREE"' )? '2,6,1':'2,64,1';
	    $info= ($topology eq '"FATTREE"' || $topology eq '"TREE"' )? 'number of last level individual router`s endpoints.' :'Number of NoC routers in row (X dimension)';
	    $type= 'Spin-button';             
	    ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,$show_noc,'noc_param',1);

    
    #Topology T2 parameter
    if($topology eq '"MESH"' || $topology eq '"FMESH"' || $topology eq '"TORUS"' || $topology eq '"FATTREE"' || $topology eq '"TREE"' ) {
        $label= ($topology eq '"FATTREE"' || $topology eq '"TREE"')?  'L' :'Routers per column';
        $param= 'T2';
        $default='2';
        $content=  ($topology eq '"FMESH"')? '1,16,1': '2,16,1';
        $info= ($topology eq '"FATTREE"' || $topology eq '"TREE"')? 'Fattree layer number (The height of FT)':'Number of NoC routers in column (Y dimension)';
        $type= 'Spin-button';             
        ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,$show_noc,'noc_param',1);
    } else {
        $mpsoc->object_add_attribute('noc_param','T2',1);        
    }
    
    #Topology T3 parameter
    if($topology eq '"MESH"' || $topology eq '"FMESH"' || $topology eq '"TORUS"' || $topology eq '"RING"' || $topology eq '"LINE"') {
    	$label="Router's endpoint number";
		$param= 'T3';
        $default='1';
        $content='1,4,1';
        $info= "In $topology topology, each router can have up to 4 endpoint processing tile.";
        $type= 'Spin-button';             
        ($row,$coltmp)=add_param_widget ($mpsoc,$label,$param, $default,$type,$content,$info, $table,$row,undef,$show_noc,'noc_param',1);
    }      
    
    
	}else{#its a custom Topology
		($row,$coltmp)=config_custom_topology_gui($mpsoc,$table,$txview,$row);
	}
	return ($row,$coltmp);

}


sub squeu_jobs_summary{
    my $input=shift;
	
	my @list;               
    @list = split("\n",$input);
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
                if($f eq "TIME"){
                  $st{$f} = $v;
                }
            }
        }
        $num++;
    }

    #print $id;
    my $t=$num-1;
    my $timestamp = localtime(time);
    my $out= "  $timestamp:\n\tTotal jobs: $t\n";
    foreach my $p (sort keys %st) {
           $out.= "\t$p: $st{$p}\n  "
    }
    $out.= "\n";
    return $out;

}



sub ps_active_job{
    my $input=shift;
	my @list;               
    @list = split("\n",$input);
    my @names;
    my $num=0;
   
    my $jobs;
    my $njob=0;
    my @job_IDs=();
    foreach my $line (@list){
        my $ID;
        $line=~ s/^\s+//;  # Remove leading spaces   
        if($num==0){    
               
            @names = split /\s+/, $line ;
            $jobs="$line\n";
        } else{
            my @values = split /\s+/, $line ;
            foreach my $f (@names){
                my $v= remove_all_white_spaces(shift(@values));
                $f=remove_all_white_spaces($f);                           
                if($f eq "TIME"){ #only report jobs with active time larger than zero                 
                   if ("$v" ne "00:00:00"){
                        $jobs.="$line\n";
                        $njob++;
                        push (@job_IDs, $ID);
                   } 
                }if($f eq "PID"){
                    $ID=$v;
                }
            }
        }
        $num++;
    }
   #print $id;    
    my $timestamp = localtime(time);
    my $out= "  $timestamp:\n\tTotal jobs: $njob\n";   
    $out.= "$jobs\n" if ($njob !=0);
    return ($out,@job_IDs);
}


sub select_hpm_metrics {
    my $set_win = def_popwin_size(50,50,"remote server setting",'percent');
	
	
	
	my $ok = def_image_button('icons/select.png','OK');
	my $table=def_table(10,10,FALSE);
	my $mtable=def_table(10,10,FALSE);	
	my $scrolled_win= add_widget_to_scrolled_win($table);
	$mtable->attach_defaults($scrolled_win,0,1,0,9);
		
	


    my $remain=30;
    my $label =gen_colored_label("Remaining HPM: $remain",17);
    $table->attach ($label,0,10,0,1,'fill','fill',2,2);
	my ($row,$col)=(0,0);
	
	$row=1;
	my $info;
	my $i=1;
	foreach my $p (@hpms){
	    my $w= gen_checkbutton();
	    ($row,$col,$w)=add_param_widget  ($self, " $i- $p", $p, '1\'b0', "Check-box", 1,undef, $table,$row,$col,1, 'HPM', 1, undef, 'vertical');        
	    $i++;
	    if (($i-1)%10==0){
	         $col+=6;
	         $row=1;
	    }
	    $w-> signal_connect("toggled" => sub{

	        if($w->get_active()) {$remain--;}
			else {$remain++ ;}
			$label->destroy;
			$label =gen_colored_label("Remaining HPM: $remain",17);
			$table->attach ($label,0,10,0,1,'fill','fill',2,2);
			$label->show_all;
				   
	    });
	}
	
#	$table->show_all;
	
	
	$ok->signal_connect("clicked"=> sub{
		#save setting
		#open(FILE,  ">$paths_file") || die "Can not open: $!";
		#print FILE Data::Dumper->Dump([\%$obj],['glob']);
		#close(FILE) || die "Error closing file: $!";
	
			
		

	
		$set_win->destroy;
	

	});
	
		
	$mtable-> attach ($ok , 0, 1,  9, 10,'expand','shrink',2,2); 
	
	$set_win->add ($mtable);
	
	$set_win->show_all;

}

  
  
 

sub gen_hpm_h {
    
   
    
    my $hpm_pin=1;
    my $sarg_counter=3;
    my $hpm_init = "";
    my $hpm_roi_st="";
    my $hpm_roi_end="";
    my $print_hpm="";
    
    foreach my $p (@hpms){
        my $val=$self->object_get_attribute('HPM',$p);
        if ($val eq "1\'b1") {
            $hpm_init.=   "    write_csr(mhpmevent$sarg_counter,   $hpm_pin);//\t$p\n";
            $hpm_roi_st.= "    sargantana_counters[$sarg_counter] = read_csr(mhpmcounter$sarg_counter);//\t$p\n";
            $hpm_roi_end.="    sargantana_counters[$sarg_counter] = read_csr(mhpmcounter$sarg_counter) - sargantana_counters[$sarg_counter];//\t$p\n";
            $print_hpm.=  "    printf(\"$p:  \%d \\n\", sargantana_counters[$sarg_counter]);\n";
            $sarg_counter++;
        }
        $hpm_pin++;
       
    }
    
    
     my $txt =
"/* -----------------------------------------------
 * Project Name   : OpenPiton + Lagarto
 * File           : all_stats.h
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Alireza Monemi
 * Email(s)       : alireza.monemi\@bsc.es
 * -----------------------------------------------*/
#ifndef __HPM_H
#define __HPM_H
#include \"util.h\"


#ifndef EXTERNAL_HPM_EVENTS
    #define EXTERNAL_HPM_EVENTS  10
#endif

static uint64_t cycles;
static uint64_t instructions;

static uint64_t sargantana_counters[31];


static void init_hpm() {\n
$hpm_init
}

uint32_t roi_start (void){
    init_hpm();
    cycles = read_csr(mcycle);
    instructions = read_csr(minstret);
$hpm_roi_st
   return 0; 
}

uint32_t roi_end (void){
    cycles = read_csr(mcycle) - cycles;
    instructions = read_csr(minstret) - instructions;
$hpm_roi_end
    return 0; 
}


uint32_t print_metrics (char *test_name ){
    printf(\"\\n\");
    printf(\"--  \%s  -- \\n\", test_name);
    printf(\"Cycles:  \%d \\n\", cycles);
    printf(\"Instructions:  \%d \\n\\n\", instructions);
$print_hpm
    return 0; 
}

#define pmu_stats(code, iter) do { \\
    roi_start(); \\
    code; \\
    roi_end(); \\
    if(argv[0][0] == nc-1) { \\
        print_metrics(stringify(code));\\
    }\\
    BARRIER();\\
} while(0)


#endif

";
    
  #save hpm.h
  my $file= abs_path(__FILE__."/../benchmark/app-scalar");
 
  $file.="/custom_hpm.h";
  print "$file\n";
  open(FILE,  ">$file") || die "Can not open: $!";
  print FILE  $txt;
  close(FILE) || die "Error closing file: $!";  
}


sub add_active_job{
	my ($id)=@_;
	$counter=100000;
	$self->object_add_attribute('CTRL',"ACTIVE","enabled");

}



Gtk3->main;
0;


