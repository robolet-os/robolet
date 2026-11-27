#!/bin/bash
# $1 = binary; prints outcome counts per world/mode/opts
B=$1
for wd in ${WORLDS:-0 1 2 3 4}; do for md in 1 0; do for op in 0 1 2 3; do
  [ $md = 0 ] && [ $op = 0 -o $op = 2 ] && continue
  out=$(OPTS=$op $B $wd $md 0 1 2 3 4 5 6 7 | grep "^MISSION")
  c=$(echo "$out" | awk -F'|' '{gsub(/ /,"",$2); print $2}' | sort | uniq -c | tr '\n' ' ')
  ct=$(echo "$out" | grep -o "contacts [0-9]*" | awk '{s+=$2} END{print s}')
  wp=$(echo "$out" | grep -o "wrong-pose driving [0-9.]*" | awk '{s+=$3} END{print s}')
  echo "world $wd mode $md opts $op : $c| contacts $ct | wrong-pose s $wp"
done; done; done
