#!/usr/bin/perl


use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/lib";
use Term::ReadKey;

use Cwd;
my $current_dir = getcwd;

my $diff_limit = 2000;  #limit diff report numbers fo comparsion between two files using cmp 

my $fin1 = $ARGV[0];
my $fin2 = $ARGV[1];
if (!defined $fin1 or !defined $fin2)  {
    print "Usage: perl compar.pl folder1 folder2.";
    exit 1;
}

unless (-d  "$fin1"){
        print "Error: Could not find $fin1. Abroting.. ";
        exit 1;
}
unless (-d "$fin2"){
        print "Error: Could not find $fin1. Abroting.. ";
       exit 1;
 }


system("mkdir -p $fin1/tmp");
system("mkdir -p $fin2/tmp");


sub append_text_to_file {
	my  ($file_path,$text)=@_;
	open(my $fd, ">>$file_path") or die "could not open $file_path: $!";
	print $fd $text;
	close $fd;
} 

sub get_diff{
    my ($f1,$f2)=@_;
    #print "cmp $f1 $f2\n";
    # Run the command and store the output in an array
    my @differences = qx(cmp -l $f1 $f2 | awk 'NR > 1000 {exit} {print}');
   
    # change bytenum to line num
    my $line_num=0;
    my $byte_counter=0;
    my @lines;
    open(my $fh, '<', $f1) or die "Could not open file '$f1': $!";
   
    foreach my $cmp_line (@differences) {
         # Extract byte number from cmp output (column 1)
         my ($byte_number) = split ' ', $cmp_line;
         while($byte_counter<$byte_number){
             my $line = <$fh>;
             $line_num++;
             my $len =length($line);
             $byte_counter+=$len;
         }
         push(@lines,$line_num);
        
    }
    my %seen;
    my @unify = grep { !$seen{$_}++ } @lines;
    return @unify;
}


my $n=0;

while (-f "$fin1/trace_hart_${n}_commit.log"){
    my $commit1 = "$fin1/trace_hart_${n}_commit.log";
    my $commit2 = "$fin2/trace_hart_${n}_commit.log";
    my $log1    = "$fin1/trace_hart_${n}.log";
    my $log2    = "$fin2/trace_hart_${n}.log";
    print "compare $commit1 with $commit2\n";
    unlink  "$fin1/tmp/n${n}";
    unlink  "$fin1/tmp/n${n}";
    
    my @diff= get_diff($commit1,$commit2);
    foreach my $f (@diff){
       my $c1= qx/sed -n '$f\{p;q\}' $commit1/;
       my $l1= qx/sed -n '$f\{p;q\}' $log1/;
       
       my $c2= qx/sed -n '$f\{p;q\}' $commit2/;
       my $l2= qx/sed -n '$f\{p;q\}' $log2/;
    
       append_text_to_file("$fin1/tmp/n${n}","line:$f\n$c1");
       append_text_to_file("$fin1/tmp/n${n}","$l1\n");
       append_text_to_file("$fin2/tmp/n${n}","line:$f\n$c2");
       append_text_to_file("$fin2/tmp/n${n}","$l2\n");
    
    }
    

$n++;
}

system("meld $fin1/tmp $fin2/tmp");

exit 1;





