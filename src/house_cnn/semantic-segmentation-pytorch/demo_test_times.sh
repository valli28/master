#!/bin/bash

# Image and model names
TEST_IMG=facade.jpg
MODEL_PATH=ade20k-resnet50dilated-ppm_deepsup
RESULT_PATH=./

ENCODER=$MODEL_PATH/encoder_epoch_20.pth
DECODER=$MODEL_PATH/decoder_epoch_20.pth

# Download model weights and image
if [ ! -e $MODEL_PATH ]; then
  mkdir $MODEL_PATH
fi
if [ ! -e $ENCODER ]; then
  wget -P $MODEL_PATH http://sceneparsing.csail.mit.edu/model/pytorch/$ENCODER
fi
if [ ! -e $DECODER ]; then
  wget -P $MODEL_PATH http://sceneparsing.csail.mit.edu/model/pytorch/$DECODER
fi
if [ ! -e $TEST_IMG ]; then
  wget -P $RESULT_PATH http://sceneparsing.csail.mit.edu/data/ADEChallengeData2016/images/validation/$TEST_IMG
fi

# Inference
arr = ''
for i in {1..500}
do
  echo "Welcome $i times"
	start=$(date +%s.%N)
	python3 -u test.py \
  --imgs $TEST_IMG \
  --cfg config/ade20k-resnet50dilated-ppm_deepsup.yaml \
  DIR $MODEL_PATH \
  TEST.result ./ \
  TEST.checkpoint epoch_20.pth
	end=$(date +%s.%N)
	runtime=$(echo "$end - $start" | bc)
	arr+=$(echo "$runtime" | bc)
done

printf '%s\n' "${arr[@]}"
