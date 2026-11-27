#!/bin/bash
# $1 = binary; all MISSION lines for every world/mode/opts, 8 seeds
B=$1
for wd in ${WORLDS:-0 1 2 3 4}; do for md in 1 0; do for op in 0 1 2 3; do
  [ $md = 0 ] && [ $op = 0 -o $op = 2 ] && continue
  OPTS=$op $B $wd $md 0 1 2 3 4 5 6 7 | grep "^MISSION" | sed 's/ | max cycle.*//'
done; done; done
