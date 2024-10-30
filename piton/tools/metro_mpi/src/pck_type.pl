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

sub pck_type_string {
    my $t=shift;
    my @pck_types = (" MSG_TYPE_RESERVED           " , #0
" MSG_TYPE_PREFETCH_REQ       " , #1
" MSG_TYPE_STORE_REQ          " , #2
" MSG_TYPE_BLK_STORE_REQ      " , #3
" MSG_TYPE_BLKINIT_STORE_REQ  " , #4
" MSG_TYPE_CAS_REQ            " , #5
" MSG_TYPE_CAS_P1_REQ         " , #6
" MSG_TYPE_CAS_P2Y_REQ        " , #7
" MSG_TYPE_CAS_P2N_REQ        " , #8
" MSG_TYPE_SWAP_REQ           " , #9
" MSG_TYPE_SWAP_P1_REQ        " , #10
" MSG_TYPE_SWAP_P2_REQ        " , #11
" MSG_TYPE_WB_REQ             " , #12
" MSG_TYPE_WBGUARD_REQ        " , #13
" MSG_TYPE_NC_LOAD_REQ        " , #14
" MSG_TYPE_NC_STORE_REQ       " , #15
" MSG_TYPE_LOAD_FWD           " , #16
" MSG_TYPE_STORE_FWD          " , #17
" MSG_TYPE_INV_FWD            " , #18
" MSG_TYPE_LOAD_MEM           " , #19
" MSG_TYPE_STORE_MEM          " , #20
" MSG_TYPE_LOAD_FWDACK        " , #21
" MSG_TYPE_STORE_FWDACK       " , #22
" MSG_TYPE_INV_FWDACK         " , #23
" MSG_TYPE_LOAD_MEM_ACK       " , #24
" MSG_TYPE_STORE_MEM_ACK      " , #25
" MSG_TYPE_NC_LOAD_MEM_ACK    " , #26
" MSG_TYPE_NC_STORE_MEM_ACK   " , #27
" MSG_TYPE_NODATA_ACK          " , #28
" MSG_TYPE_DATA_ACK            " , #29
" MSG_TYPE_ERROR               " , #30
" MSG_TYPE_LOAD_REQ            " , #31
" MSG_TYPE_INTERRUPT_FWD       " , #32
" MSG_TYPE_INTERRUPT           " , #33
" MSG_TYPE_L2_LINE_FLUSH_REQ   " , #34
" MSG_TYPE_L2_DIS_FLUSH_REQ    " , #35
" MSG_TYPE_AMO_ADD_REQ         " , #36
" MSG_TYPE_AMO_AND_REQ         " , #37
" MSG_TYPE_AMO_OR_REQ          " , #38
" MSG_TYPE_AMO_XOR_REQ         " , #39
" MSG_TYPE_AMO_MAX_REQ         " , #40
" MSG_TYPE_AMO_MAXU_REQ        " , #41
" MSG_TYPE_AMO_MIN_REQ         " , #42
" MSG_TYPE_AMO_MINU_REQ        " , #43
" MSG_TYPE_AMO_ADD_P1_REQ      " , #44
" MSG_TYPE_AMO_AND_P1_REQ      " , #45
" MSG_TYPE_AMO_OR_P1_REQ       " , #46
" MSG_TYPE_AMO_XOR_P1_REQ      " , #47
" MSG_TYPE_AMO_MAX_P1_REQ      " , #48
" MSG_TYPE_AMO_MAXU_P1_REQ     " , #49
" MSG_TYPE_AMO_MIN_P1_REQ      " , #50
" MSG_TYPE_AMO_MINU_P1_REQ     " , #51
" MSG_TYPE_AMO_ADD_P2_REQ      " , #52
" MSG_TYPE_AMO_AND_P2_REQ      " , #53
" MSG_TYPE_AMO_OR_P2_REQ       " , #54
" MSG_TYPE_AMO_XOR_P2_REQ      " , #55
" MSG_TYPE_AMO_MAX_P2_REQ      " , #56
" MSG_TYPE_AMO_MAXU_P2_REQ     " , #57
" MSG_TYPE_AMO_MIN_P2_REQ      " , #58
" MSG_TYPE_AMO_MINU_P2_REQ     " , #59
" MSG_TYPE_LR_REQ              " , #60
" MSG_TYPE_LOAD_NOSHARE_REQ    " , #61
" MSG_TYPE_SWAPWB_REQ          " , #62
" MSG_TYPE_SWAPWB_P1_REQ       " , #63
" MSG_TYPE_SWAPWB_P2_REQ       " , #64
);
 
 return $pck_types [$t];

}

sub hex_string{
    my $in=shift;
    return sprintf("%X",$in) ;
}

my @flit0 = (
    {name=>'MSG_OPTIONS_1' , size=>6},
    {name=>'MSG_MSHRID'    , size=>8}, 
    {name=>'MSG_TYPE'      , size=>8 , conv=>\&pck_type_string},
    {name=>'MSG_LENGTH'    , size=>8},
    {name=>'MSG_DST_FBITS' , size=>4},
    {name=>'MSG_DST_Y'     , size=>8},
    {name=>'MSG_DST_X'     , size=>8},
    {name=>'MSG_DST_CHIPID', size=>14},
);
my @flit1 = (
    {name=>'MSG_AMO_MASK0'       , size=>8 }, 
    {name=>'MSG_DATA_SIZE'       , size=>3 }, 
    {name=>'MSG_CACHE_TYPE'      , size=>1 }, 
    {name=>'MSG_SUBLINE_VECTOR'  , size=>4 }, 
    {name=>'MSG_ADDR'            , size=>48, conv=>\&hex_string}, 
);

my @flit2 = (
    {name=>'MSG_AMO_MASK1'        , size=> (135-128)+1 },
    {name=>'MSG_LSID'             , size=> (147-142)+1 },
    {name=>'MSG_SDID'             , size=> (157-148)+1 },
    {name=>'MSG_SRC_FBITS'        , size=> (161-158)+1 },
    {name=>'MSG_SRC_Y'            , size=> (169-162)+1 },
    {name=>'MSG_SRC_X'            , size=> (177-170)+1 },
    {name=>'MSG_SRC_CHIPID'       , size=> (191-178)+1 },
);

sub decode_pck  {
	my ($noc,$num, $hexvalue)=@_;
	my @chunks = $hexvalue =~ /(.{1,8})/g;

    my $dec="";
	my $int32_2 = hex ($chunks[0]);
    my $int32_1 = hex ($chunks[1]);	
	my $int64 = (($int32_2 << 32) + $int32_1);
    return decode_pck_f ($int64, \@flit0) if ($num == 0);
	return decode_pck_f ($int64, \@flit1) if ($num == 1);
    return decode_pck_f ($int64, \@flit2) if ($num == 2);
	return $dec; 
}

sub decode_pck_f {
    my $int64=shift;
    my $ref= shift;
    my @flit =@{$ref};
    my $d="";
    foreach my $f (@flit){
       $d.= "$f->{name} ";
       my $mask=(1<<$f->{size})-1;
       my $val=$int64 & $mask;
       $int64 >>= $f->{size};
       if (defined $f->{conv}){
            my $convert_function = $f->{conv};
            # Call the function using the reference
            $val = $convert_function->($val);
       }
       $d.="      = $val\n";
    }
    return $d;
}



