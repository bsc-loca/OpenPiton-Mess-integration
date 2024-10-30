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
use Term::ReadKey;

use Cwd;
my $current_dir = getcwd;

my $diff_limit = 2000;  #limit diff report numbers fo cor mparsion between two files using cmp 

my $fin1 = $ARGV[0];
my $fin2 = $ARGV[1];
if (!defined $fin1 or !defined $fin2)  {
    print "Usage: perl compar.pl folder1 folder2.";
    exit 1;
}


sub extract_pck {
    my ($fin1,$fin2)=@_;
    #check if $fin1/transcript $fin2/transcript exsited
    unless (-f  "$fin1/transcript"){
        print "Error: Could not find $fin1/transcript. Abroting.. ";
        exit 1;
    }
    unless (-f  "$fin2/transcript"){
        print "Error: Could not find $fin1/transcript. Abroting.. ";
        exit 1;
    }
    
    unless(-d  "$fin2/nocs"){
        print "Extract each NoCs input output packets in $fin1/transcript\n";
        system ("perl $current_dir/pck_sep.pl $fin1/transcript  $fin1/nocs");
    }
    unless(-d  "$fin2/nocs"){
        print "Extract each NoCs input output packets in $fin2/transcript\n";
        system ("perl $current_dir/pck_sep.pl $fin2/transcript $fin2/nocs");
    }
}

sub get_nocs_files{
    # Specify the directory to list files from
    my $dir = shift;
    opendir(my $dh, $dir) or die "Could not open '$dir': $!";
    my @files = grep { -f "$dir/$_" && !/^\Qline\E/ && !/\.tmp$/ } readdir($dh);
    closedir($dh);   
    return @files;
}


sub get_diff{
    my ($f1,$f2)=@_;
    print "cmp $f1 $f2\n";
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
    return \@unify;
}

sub get_diff_nocs {
    my ($f1,$f2)=@_;
    my @nocs = get_nocs_files("$fin1/nocs");
    my %diffs;
    foreach my $p (@nocs) {
        $diffs{$p}=get_diff ("$fin1/nocs/$p","$fin2/nocs/$p");    
    }
    return %diffs
}

sub get_first_diff {
    my $ref=shift;
    my %heads=%{$ref};
    # Initialize variables to track the minimum value and key
    my $min_key;
    my $min_value = undef;  # Use undef to check if we have set a value

    # Iterate over the hash to find the minimum value and its key
    while (my ($key, $value) = each %heads) {
        if (not defined $min_value ) {
            $min_value = $value;
            $min_key = $key;
        }
        elsif (defined $value && $value < $min_value) {
            $min_value = $value;
            $min_key = $key;
        }
    }
    return ($min_key,$min_value);
}

sub remove_the_last_diff {
    my @numbers=@_;
    my @new_array;
    # Initial last number
   

    # Loop until we find a number that is one greater than the last one
    while (@numbers) {
        my $next_num =$numbers[1];
        return undef if !defined $next_num; 
        my $current_num = shift @numbers;  # Remove the first element

        # The $next_num is larger than $current_num+1 stop triming
        if ($current_num+1 < $next_num ) {            
            
            last;  # Exit the loop
        }
        
       
    }
    return @numbers;

}




sub show_diff{
    my ($fin1,$fin2,$noc_name,$line,$diff_num)=@_;
    # Turn off input buffering to capture a single keypress
    $line-=10;
     print "./meld_compar.sh -n 100 -s $line -t $line $fin1/nocs/$noc_name  $fin2/nocs/$noc_name\n";
    
    system( "bash ./meld_compar.sh -n 100 -s $line -t $line $fin1/nocs/$noc_name  $fin2/nocs/$noc_name\n");
    print "Press c to exit or any other key to continue...\n";
    # Reset terminal mode back to normal
    # Wait for a single keypress
    ReadMode('raw');  # Set the terminal to raw mode
    my $key = ReadKey(0);  # Read a single key (non-blocking)
    ReadMode('restore');  # Restore the terminal to its original state 
    exit 0 if ($key eq 'c');   
}







#Extract each NoCs input output packets 
extract_pck($fin1,$fin2);
#Extract the line numbers where each NoCs input output packets 
my %diff_lines = get_diff_nocs($fin1,$fin2);
my @nocs = get_nocs_files("$fin1/nocs");


my %heads;
my %nocs_head;
my $end=0;
my $diff_num=0;
while($end==0){
    $end=1;
    foreach my $n (@nocs){
        if(defined $diff_lines{$n}){
            
            my @a=@{$diff_lines{$n}};
            my $lin_noc=$a[0];  
            if(defined  $lin_noc){
                $end=0;
                my $d= qx/sed -n '$lin_noc\{p;q\}' $fin1\/nocs\/lines_$n/;
                $d =~ s/^\s+|\s+|\n//g; 
                $heads{$n}=$d; 
                $nocs_head{$n}=$lin_noc;
                #print "$n :$lin_noc $heads{$n}\n"; 
            }else{
                 $heads{$n}=undef;  
            }    
            
        }else {
            $heads{$n}=undef; 
        }       
    }
    my ($min_key,$min_value) = get_first_diff(\%heads);
    $nocs_head{$min_key};
   # print " ($min_key,$min_value,$nocs_head{$min_key})\n";
    my @a=@{$diff_lines{$min_key}};
    my @new=remove_the_last_diff(@a);
    $diff_lines{$min_key}=\@new;
    $diff_num++;
    show_diff($fin1,$fin2,$min_key,$nocs_head{$min_key},$diff_num);
}

print "done!\n";

