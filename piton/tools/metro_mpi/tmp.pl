#!/usr/bin/perl 

use FindBin;
use lib $FindBin::Bin;

use strict;
use warnings;

use lib '../lib';


use Capture::Tiny ':all';
use File::Basename;
use Getopt::Long;
use base 'Class::Accessor::Fast';
use Cwd;
use Cwd 'abs_path';
use Getopt::Std;

use List::MoreUtils qw(uniq);

use GD::Graph::bars;


sub save_file {
	my  ($file_path,$text)=@_;
	open my $fd, ">$file_path" or die "could not open $file_path: $!";
	print $fd $text;
	close $fd;	
} 

use Chart::Gnuplot;

my $out_file ="./test.png";
my %graphs_info;
$graphs_info{X_Title}="AAA";
$graphs_info{Y_Title}="BBB";
$graphs_info{G_Title}="CCC";


my $chart = Chart::Gnuplot->new(
    	output => "$out_file",
		xlabel => $graphs_info{X_Title},
	    ylabel => $graphs_info{Y_Title},
	    title   => $graphs_info{G_Title},
		terminal=> 'png',
		
	);


	# Raw data
my @x = qw(A B C D E F);
my @y1 = (1, 8, 3, 2, 4, 4);
my @y2 = (2, 2, 5, 1, 7, 6);
my @y3 = (4, 3, 2, 2, 3, 5);

# DataSet object of the 1st data set
my $h1 = Chart::Gnuplot::DataSet->new(
    xdata => \@x,
    ydata => \@y1,
    title => "1st data set",
    fill  => {density => 0.2},
    style => "histograms",
);

# DataSet object of the 2nd data set
my $h2 = Chart::Gnuplot::DataSet->new(
    xdata => \@x,
    ydata => \@y2,
    title => "2nd data set",
    color => "dark-green",
    fill  => {density => 0.2},
    style => "histograms",
);

# DataSet object of the 3rd data set
my $h3 = Chart::Gnuplot::DataSet->new(
    xdata => \@x,
    ydata => \@y3,
    title => "3rd data set",
    fill  => {density => 0.2},
    style => "histograms",
);

# Plot the graph
$chart->plot2d($h1, $h2, $h3);



