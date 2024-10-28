#!/usr/bin/perl


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
