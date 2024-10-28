#!/usr/bin/perl

package GUI;
use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/lib";
use File::Basename;
use Getopt::Long;
use base 'Class::Accessor::Fast';
use Cwd;
use Cwd 'abs_path';
use Getopt::Std;
use List::Util qw(max sum min);
use feature qw(switch);
require "mpi_src.pl";





 wait_until_squeue_is_empty();
