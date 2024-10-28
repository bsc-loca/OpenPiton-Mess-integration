

#sims -sys=manycore -x_tiles=1 -y_tiles=1 -vlt_build -ariane -config_rtl=MINIMAL_MONITORING -config_rtl=PITON_NO_CHIP_BRIDGE 


rm isa_test_result.csv
echo "ISA Test, Resuilt"

while IFS="" read -r p || [ -n "$p" ]
do
  printf 'Running ISA TEST: %s\n' "$p"
  sims -sys=manycore -vlt_run -x_tiles=1 -y_tiles=1 $p.S -ariane -precompiled -rtl_timeout 100000000
  if grep 'Simulation -> PASS (HIT GOOD TRAP)' status.log
  then
    echo "$p, PASS" >> isa_test_result.csv
  else
    echo "$p, FAILED" >> isa_test_result.csv
  fi

done < isa_test_list.txt
