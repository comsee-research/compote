#!/bin/bash

dataset=$1
path=$2
features=$3

camera="../camera.js"
if [ -f "../${dataset}/camera.js" ]; then
	camera="../${dataset}/camera.js"
fi

params=""
if [ -f "../${dataset}/params.js" ]; then
	params="../${dataset}/params.js"
fi

./${path}/calibrate \
	--pcamera "${camera}" \
	--pimages "../${dataset}/images.js" \
	--pscene "../${dataset}/scene.js" \
	--pparams "${params}" \
	--gui false \
	--verbose true -l 15 \
	--output "../${dataset}/camera_optimized.js"

