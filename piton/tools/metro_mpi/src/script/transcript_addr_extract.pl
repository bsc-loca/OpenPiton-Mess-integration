#!/usr/bin/perl


use FindBin;
use lib $FindBin::Bin;
use strict;
use warnings;
use lib "$FindBin::Bin/lib";


my @section_dev=(
    "----------",
    "L15_MON_END"
);

my $fin = $ARGV[0];
my $addr =$ARGV[1]; #E.g "80001210";
my $out_dir =( defined $ARGV[2]) ? $ARGV[2] : "./out";
my $fout1 ="$out_dir/sections.txt";
my $fout2 ="$out_dir/flow.txt";

system ("mkdir -p $out_dir");

if (!defined $fin) {
    print "Error: No input file is given.\n";
    help();
}
if (!defined $addr) {
    print "Error: No input address is given.\n";
    help();
}

sub help{
  
    print << "END_USAGE";
Usage:   perl $0 FILE ADDRESS [OUT_DIR]
   FILE:     The transcript file containing the OpenPiton monitored messages
   ADDRESS:  The specific address you want to track
   OUT_DIR:  The output folder name where the results are stored (default: out).
END_USAGE
    exit(1);

}





sub append_text_to_file {
	my  ($file_path,$text)=@_;
	open(my $fd, ">>$file_path") or die "could not open $file_path: $!";
	print $fd $text;
	close $fd;
}


sub print_section {
    my ($file_path, $addr, @mathes) = @_;
    unlink $fout1;
    my $found=0;
    my $sesseion="############# 0 ################\n";
    my $num=0;
    open my $file, '<', $file_path or die "print_section Cannot open file:$file_path $!\n";
    while (my $line = <$file>) {
        my $match=0;
        foreach my $p (@mathes){ $match=1 if ($line =~ /$p/);}
        if ($match) {  # Matches "------" or L15_MON_END                 
            append_text_to_file ("$fout1",$sesseion ) if($found);
            $found=0;
	     
            $sesseion="############# $num ################\n";
        }
        $sesseion.=$line;
        if ($line =~ /$addr/) {
            $num++ if($found ==0);
            $found=1;
           
        }
    }  

    close $file;
}


sub extract_info {
    my ($file_path, $addr) = @_;
    unlink $fout2;
    
    open my $file, '<', $file_path or die "extract_info Cannot open file:$file_path $!\n";
    append_text_to_file ("$fout2","Tile#, OP, addr" ); 
    while (my $line = <$file>) {
        my @fileds = split /\s+/, $line;
        my $type = $fileds[6];
        # 34846000 TILE0 L1.5 th16: Received PCX PCX_REQTYPE_STORE   Addr 0x0080001210, nc 0, size 3, invalall 0, pf 0, bs 0, bsi 0, l1way 0
        if ($line =~ /\s+TILE(\d+)\s+(.*?)PCX(.*?)Addr\s+0x(\w+)/) {
            my $addr_hex = hex($4);
            my $opc=$3;
            my $tile=$1;
            append_text_to_file ("$fout2", "T$tile, $opc, $addr_hex\n") if(hex($addr)==$addr_hex);
        }
        # 34851000 TILE0 L1.5: Sending NOC1      L15_NOC1_REQTYPE_ST_FILL_REQUEST   mshrid 3, nc 0, size 7, pf 0, address 80001210
        if ($line =~ /\s+TILE(\d+)\s+(.*?)NOC(\w+)\s+(\w+)\s+(.*?)address\s+(\w+)/){
             next if ($2 =~/L1.5: Received/);
             my $tile=$1;
             my $opt=$4;
             my $addr_hex = hex($6);
             
             append_text_to_file ("$fout2", "t$tile,  $opt, $addr_hex\n") if(hex($addr)==$addr_hex);
        }
   }    

}


my $file_path = "$fin";  # Replace with the actual file path

print_section($file_path,$addr,@section_dev);
extract_info($fout1,$addr);


