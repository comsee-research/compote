#!/bin/bash

exe="./src/calibrate/calibrate"
path="../examples"

camera="${path}/config/camera.js"
params="${path}/config/params.js"
scene="${path}/config/scene.js"

features="${path}/obs/linked-observations-R12-A.bin.gz"

${exe} \
	--pcamera "${camera}" \
	--pscene "${scene}" \
	--pparams "${params}" \
	--features "${features}" \
	--gui false \
	--verbose true -l 15 \
	--output "camera-optimized.js" \

