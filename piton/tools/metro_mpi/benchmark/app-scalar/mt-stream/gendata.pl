#!/usr/bin/perl -w
#==========================================================================
# gendata.pl
#
#
(our $usageMsg = <<'ENDMSG') =~ s/^\#//gm;
#
# Simple script which creates an input data set and the reference data
# for the stream.
#
ENDMSG

use strict "vars";
use warnings;
no  warnings("once");
use Getopt::Long;

 my $array_alignment =64;

#--------------------------------------------------------------------------
# Command line processing
#--------------------------------------------------------------------------

our %opts;

sub usage()
{

  print "\n";
  print " Usage: gendata.pl [options] \n";
  print "\n";
  print " Options:\n";
  print "  --help  print this message\n";
  print "  --size  size of input data [1000]\n";
  print "  --pause number of nop between copy operands [0] \n";
  print "  --loop  number of repeating stream functions [1] \n";
  print "  --RD_ratio read to write ratio [0]\n";
  print "  --seed  random seed [1]\n";
  print "  --Dtype Data type [double] , float, int\n";
  print "$usageMsg";

  exit();
}

sub processCommandLine()
{

  $opts{"help"} = 0;
  $opts{"size"} = 1000;
  $opts{"seed"} = 1;
  $opts{"Dtype"} = "double";
  $opts{"pause"} = 0;
  $opts{"loop"}=1;
  $opts{"RD_ratio"} = 0;
  
  Getopt::Long::GetOptions( \%opts, 'help|?', 'size:i', 'seed:i', 'Dtype:s', 'pause:i', 'loop:i', 'RD_ratio=i' ) or usage();
  $opts{"help"} and usage();
  
  
  
  
  
  my @types=('double','int','float');
  unless ( grep( /^$opts{Dtype}$/, @types ) ) {
     print "Error: $opts{Dtype} is an unsupported data type. Valid values are @types\n";
     exit ();
  } 

}

#--------------------------------------------------------------------------
# Helper Functions
#--------------------------------------------------------------------------

sub printArray
{
  my $arrayName = $_[0];
  my $arrayRef  = $_[1];

  my $numCols = 20;
  my $arrayLen = scalar(@{$arrayRef});

  print  "static data_t ".$arrayName."[ARRAY_SIZE] __attribute__((aligned ($array_alignment))) = \n";
  print  "{\n";

  if ( $arrayLen <= $numCols ) {
    print  "  ";
    for ( my $i = 0; $i < $arrayLen; $i++ ) {
      print  sprintf("%g",$arrayRef->[$i]);
      if ( $i != $arrayLen-1 ) {
        print  ", ";
      }
    }
    print  "\n";
  }

  else {
    my $numRows = int($arrayLen/$numCols);
    for ( my $j = 0; $j < $numRows; $j++ ) {
      print  "  ";
      for ( my $i = 0; $i < $numCols; $i++ ) {
        my $index = $j*$numCols + $i;
        print  sprintf("%g",$arrayRef->[$index]);
        if ( $index != $arrayLen-1 ) {
          print  ", ";
        }
      }
      print  "\n";
    }

    if ( $arrayLen > ($numRows*$numCols) ) {
      print  "  ";
      for ( my $i = 0; $i < ($arrayLen-($numRows*$numCols)); $i++ ) {
        my $index = $numCols*$numRows + $i;
        print  sprintf("%g",$arrayRef->[$index]);
        if ( $index != $arrayLen-1 ) {
          print  ", ";
        }
      }
      print  "\n";
    }

  }

  print   "};\n\n";
}









#--------------------------------------------------------------------------
# Main
#--------------------------------------------------------------------------

sub main()
{
 
  my $in_max =5000; #should be int
  processCommandLine();
  srand($opts{"seed"});
  my $Alfa=  ($opts{Dtype} eq 'int')? 1 : 0.1;
 
  my @A;
  my @B;
  
  # create random input arrays
  my $a;
  my $b;
  for ( my $i = 0; $i < $opts{"size"}; $i++ ) {   
      $a->[$i] = int(rand($in_max));
      $b->[$i] = int(rand($in_max));      
      
      push( @A, $a->[$i] *$Alfa );
      push( @B, $b->[$i] *$Alfa );  
  }
	

  

  print  "\n#ifndef __DATASET_H";
  print  "\n#define __DATASET_H";
  print  "\n\#define ARRAY_SIZE ".($opts{"size"})." \n";
 
  print  "\n\#define DATA_TYPE\n";  
  print  "\ntypedef $opts{Dtype} data_t;\n";
  
  print  "#define PAUSE $opts{pause}\n";
  print  "#define LOOP $opts{loop}\n";
  print  "#define RD_RATIO $opts{RD_ratio}\n";
  print  "#define ARRAY_ALIGNMENT $array_alignment\n";




  printArray( "input1_data", \@A );
  printArray( "input2_data", \@B );

  #printArray( "verify_data", \@A );
 

  print  "\n#endif //__DATASET_H";
 
}

main();

