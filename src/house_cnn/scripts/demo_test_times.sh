#!/bin/bash


# Inference
arr = ''
for i in {1..500}
do
  echo "Welcome $i times"
	start=$(date +%s.%N)
	python3 house_test.py
	end=$(date +%s.%N)
	runtime=$(echo "$end - $start" | bc)
	arr+=$(echo "$runtime" | bc)
done

printf '%s\n' "${arr[@]}"
