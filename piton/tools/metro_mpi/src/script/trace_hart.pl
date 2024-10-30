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



my $input_file = "$fin";  # Replace with the actual file path
my $output_file= "$out_dir/trace_log";

# Open the input file for reading
open my $input_fh, '<', $input_file or die "Could not open $input_file: $!";

# Open the output file for writing
open my $output_fh, '>', $output_file or die "Could not open $output_file: $!";

# Process each line in the input file
while (my $line = <$input_fh>) {
    # Remove the first three columns using a regular expression
    $line =~ s/^\s*\S+\s+\S+\s+\S+\s+//;

    # Print the modified line to the output file
    print $output_fh $line;
}

# Close the file handles
close $input_fh;
close $output_fh;

print "File successfully processed. Output saved to $output_file.\n";
