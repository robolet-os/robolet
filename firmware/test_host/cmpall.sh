#!/bin/bash
# node version vs single loop for every world/mode/opts, seeds 0-7; prints one line per case and a total
B=${1:-./tmn}; bad=0; n=0
for wd in ${WORLDS:-0 1 2 3 4}; do for md in 1 0; do for op in 0 1 2 3; do
  [ $md = 0 ] && [ $op = 0 -o $op = 2 ] && continue
  out=$(OPTS=$op $B $wd $md ${SEEDS:-0 1 2 3 4 5 6 7})
  echo "$out" | grep "^world"
  echo "$out" | grep -A2 "differs" | head -6
  n=$((n + $(echo "$out" | grep -c "^world"))); bad=$((bad + $(echo "$out" | grep -c "NOT the same")))
done; done; done
echo "TOTAL $n cases, $bad differ"
