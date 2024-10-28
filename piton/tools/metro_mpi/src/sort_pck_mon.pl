#!/usr/bin/perl 

use FindBin;
use lib $FindBin::Bin;
#use File::Copy;
#use File::Copy::Recursive qw(fcopy rcopy dircopy fmove rmove dirmove);

use strict;
use warnings;

use File::Basename;

use Cwd;


my $dirname = $FindBin::Bin;



my $flist_in="$dirname/flist";
my $flist_out="$dirname/flist_local";
my $src_hdl = "$dirname/src_hdl";

my $fbuild_in="$dirname/build.sh";
my $fbuild_out="$dirname/build_local.sh";
my $src_sw = "$dirname/src_sw";


system ("rm -rf $src_hdl; mkdir $src_hdl;");
system ("rm -rf $src_sw; mkdir $src_sw;");



#modify build.sh
open(FLIST_IN,  $fbuild_in) or die("Could not open $fbuild_in.");
open(FLIST_OUT, '>', $fbuild_out) or die("Could not open $fbuild_out.");

my $n=0;
foreach my $line (<FLIST_IN>)  {  
    chomp($line); 
	my @keys = split (/\s+/ , $line);
	foreach my $k (@keys){
		
		if($k eq "flist"){
			print  FLIST_OUT  "flist_local ";
		}
		#check if its a file
		elsif (-f $k){			
			my $name=basename($k);
        	my $new_file="$src_sw/$name";
        	if (-f $new_file){
        	    die("$new_file is already exsited in $src_sw\n.");
        	}
        	fcopy ($k ,$new_file) or die "Copy failed: $!";
        	print  FLIST_OUT  "$new_file ";
			
		}
		#check if its a folder
		elsif($k =~ /^-I/){
			my $include=$k;
        	$include =~ s/^-I//g;
        	my $name=basename($include);        	
            my $s=Cwd::abs_path($include);
            $s="" if !defined $s; 
        	if ($s eq Cwd::abs_path($dirname)){
        	    print  FLIST_OUT  "\+incdir\+$dirname\n";
        	}else{
        	    my $new_folder="$src_sw/f$n/$name";
        	    dircopy ($include, $new_folder) if (-d $include);
            	print  FLIST_OUT  "-I$new_folder ";
            	$n++;
        	}
		}		
		else{
			print  FLIST_OUT  "$k ";
		}
	}
	print FLIST_OUT "\n";
}

close(FLIST_IN);
close(FLIST_OUT);


#modify flistfile

open(FLIST_IN,  $flist_in) or die("Could not open $flist_in.");
open(FLIST_OUT, '>', $flist_out) or die("Could not open $flist_out.");

$n=0;
foreach my $line (<FLIST_IN>)  {   
    
    chomp($line);
    #remove -v -sv
    $line =~ s/^\s*-v\s+|^\s*-sv\s+//g;
    #remove comments
    $line =~ s/\s+\/\/.*//g;
	$line =~ s/^\s*\/\/.*//g;
	if ($line =~ /^\s*$/) {
   	 # "The line contains only whitespace. ignore it";
	}
    elsif ($line =~ /^\s*\+incdir\+/){
        my $include=$line;
        $include =~ s/^\s*\+incdir\+//g;
        my $name=basename($include);
        if (Cwd::abs_path($include) eq Cwd::abs_path($dirname)){
            print  FLIST_OUT  "\+incdir\+$dirname\n";
        }else{
            my $new_folder="$src_hdl/f$n/$name";
            dircopy ($include,$new_folder) if (-d $include);
            print  FLIST_OUT  "\+incdir\+$new_folder\n";
            $n++;
        }

    }
    #next;

    elsif (-f $line){
        
        my $name=basename($line);
        my $new_file="$src_hdl/$name";
        if (-f $new_file){
        	#a file with the same name as the new_file is already exsited in $src_hdl. we need to copy it in another folder
            $new_file="$src_hdl/f$n/$name";
            system ("mkdir -p $src_hdl/f$n");
            $n++;
        }
        fcopy ($line ,$new_file) or die "Copy failed: $!";
        print  FLIST_OUT  "$new_file\n";

    }
    else{
        die("cannot convert $line to the new filelist\n.");
    }
}
close(FLIST_IN);
close(FLIST_OUT);



sub fcopy {
    my ($src,$dest)=@_;    
    system("cp $src $dest") == 0 or die "system fcopy failed: $?";
    return 1;
}

sub dircopy {
  my ($src,$dest)=@_;    
  system("mkdir -p $dest; rsync -a   $src/* $dest/  --exclude=spike.so") == 0 or die "system fcopy failed: $?";
  return 1;
}
