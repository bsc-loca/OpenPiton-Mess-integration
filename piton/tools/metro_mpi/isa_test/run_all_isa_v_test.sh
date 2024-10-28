


#sims -sys=manycore -x_tiles=1 -y_tiles=1 -vlt_build -ariane -config_rtl=MINIMAL_MONITORING -config_rtl=PITON_NO_CHIP_BRIDGE -config_l15_l1d_cacheline_size=32

rm isa_v_test_result.csv
echo "ISA Test, Resuilt"

while IFS="" read -r p || [ -n "$p" ]
do
  printf 'Running ISA TEST: %s\n' "$p"
  sims -sys=manycore -vlt_run -x_tiles=1 -y_tiles=1 $p.S -ariane -precompiled -trap_offset=0x80000000 -rtl_timeout=1000000 
  if (grep "Simulation -> PASS (HIT GOOD TRAP)" status.log)
  then
    echo "$p, PASS" >> isa_v_test_result.csv 
  else
    echo "$p, FAILED" >> isa_v_test_result.csv
  fi

done < isa_v_test_list.txt
