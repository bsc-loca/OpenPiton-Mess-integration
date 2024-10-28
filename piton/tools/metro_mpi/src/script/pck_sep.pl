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

