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


my $fin = $ARGV[0];
if (!defined $fin) {
    print "Error. No input file is given.";
    exit 1;
}

my $out_dir =( defined $ARGV[1]) ? $ARGV[1] : "./out";
system ("rm -rf $out_dir; mkdir $out_dir;");

sub append_text_to_file {
	my  ($file_path,$text)=@_;
	open(my $fd, ">>$file_path") or die "could not open $file_path: $!";
	print $fd $text;
	close $fd;
}


sub print_section {
    my ($file_path, $section_name) = @_;

    open my $file, '<', $file_path or die "Cannot open file: $!\n";

    my $in_section = 0;
    my @current_section;
    my $line_num=0;
    while (my $line = <$file>) {
        $line_num++;
        if ($line =~ /^# -+/) {
           # print $current_section[0];
            if ($in_section && @current_section ) {
                my $input_string = $current_section[0];
                # Remove the first 7 characters
                $input_string =~ s/^.{7}//;
                # Replace spaces with hyphens
                $input_string =~ s/\s+/-/g;
                append_text_to_file ("$out_dir/$input_string","----------------------\n" );
                foreach my $p (@current_section) {
                    append_text_to_file ("$out_dir/$input_string",$p );                  
                }
                for (my $n=scalar(@current_section);$n>=0;$n--){
                    my $s=$line_num-$n;
                    append_text_to_file ("$out_dir/lines_$input_string","$s\n" );
                }
            }
            $in_section = !$in_section;
            @current_section = ();
        } elsif ($in_section) {
            push @current_section, $line;
        }
    }

    close $file;
}

my $file_path = "$fin";  # Replace with the actual file path
my $section_name_to_print = "N3 in  id:01";

print_section($file_path, $section_name_to_print);

