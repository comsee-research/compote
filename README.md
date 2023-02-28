![banner-logo](doc/imgs/banner-compote.png)

---

COMPOTE: Calibration Of Multi-focus PlenOpTic camEra.
=====================================================

COMPOTE is a set of tools to pre-calibrate and calibrate (multifocus) plenoptic cameras (e.g., a Raytrix R12) based on the [libpleno].


Quick Start
===========

### Pre-requisites

The COMPOTE applications have a light dependency list:

 * [boost] version 1.54 and up, portable C++ source libraries,
 * [libpleno], an open-souce C++ library for plenoptic camera,
 
and was compiled and tested on:
 * Ubuntu 18.04.4 LTS, GCC 7.5.0, with Eigen 3.3.4, Boost 1.65.1, and OpenCV 3.2.0,
 * Ubuntu 20.04.5 LTS, GCC 9.4.0, with Eigen 3.3.7, Boost 1.71.0, and OpenCV 4.2.0.
  
### Compilation & Test

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should compile the applications on your system.

```
mkdir build && cd build
cmake ..
make -j6
```

To test the `calibrate` application you can use the example script from the build directory:
```
./../example/run_calibration.sh
```

Applications
============

### Configuration

All applications use _.js (json)_ configuration file. The path to this configuration files are given in the command line using _boost program options_ interface.

**Options:**

| short 	| long 			| default 			| description 								|
|-------	|------			|:---------:			|:-----------:								|
| -h 		| -\-help  		|           		| Print help messages						|
| -g 		| -\-gui  		| `true`          	| Enable GUI (image viewers, etc.)			|
| -v 		| -\-verbose 	| `true`          	| Enable output with extra information		|
| -l 		| -\-level  	| `ALL` (15)       	| Select level of output to print (can be combined): NONE=0, ERR=1, WARN=2, INFO=4, DEBUG=8, ALL=15 |
| -i 		| -\-pimages 	|                	| Path to images configuration file |
| -c 		| -\-pcamera 	|                	| Path to camera configuration file |
| -p 		| -\-pparams 	| `"internals.js"` 	| Path to camera internal parameters configuration file |
| -s 		| -\-pscene  	|                	| Path to scene configuration file |
| -f 		| -\-features	| `"observations.bin.gz"`	| Path to observations file |
| -e 		| -\-extrinsics | `"extrinsics.js"` | Path to save extrinsics parameters file |
| -o 		| -\-output  	| `"intrinsics.js"`	| Path to save intrinsics parameters file |

For instance to run calibration:
```
./src/calibrate/calibrate -i images.js -c camera.js -p params.js -f observations.bin.gz -s scene.js -g true -l 7
```

Configuration file examples are given for the dataset `R12-A` in the folder `examples/`. 

### Pre-calibration

`precalibrate` uses whites raw images taken at different aperture to calibrate the Micro-Images Array (MIA) and computes the _internal parameters_ used to initialize the camera and to detect the _Blur Aware Plenoptic (BAP)_ features.

**Requirements:** minimal camera configuration, white images.

**Output:** radii statistics (.csv), internal parameters, initial camera parameters.


### Features Detection

`detect` extracts the newly introduced _Blur Aware Plenoptic (BAP)_ features in checkerboard images.

**Requirements:** calibrated MIA, internal parameters, checkerboard images, and scene configuration.

**Output:** micro-image centers and _BAP_ features.

### Camera Calibration

`calibrate` runs the calibration of the plenoptic camera (set `I=0` to act as pinholes array, or `I>0` for multifocus case). It generates the intrinsics and extrinsics parameters.

**Requirements**: calibrated MIA, internal parameters, features and scene configuration. If none are given all steps are re-done.

**Output:** error statistics, calibrated camera parameters, camera poses.

### Extrinsics Estimation (+ Calibration Evaluation)

`extrinsics` runs the optimization of extrinsics parameters given a calibrated camera and generates the poses.

**Requirements**: _internal parameters_, features, calibrated camera and scene configuration.

**Output:** error statistics, estimated poses.

COMPOTE also provides two applications to run stats evaluation on the optimized poses optained with a constant step linear translation along the _z_-axis:
 * `linear_evaluation` gives the absolute errors (mean + std) and the relative errors (mean + std) of translation of the optimized poses,
 * `linear_raytrix_evaluation` takes `.xyz` pointcloud obtained by _Raytrix_ calibration software and gives the absolute errors (mean + std) and the relative errors (mean + std) of translation.

**Note:** those apps are legacy and have been moved and generalized in the [BLADE] app's `evaluate`. 
If you want to enable the compilation of legacy applications for evaluations, add the option `-DCOMPILE_LEGACY_EVAL` to cmake.

### Blur Proportionality Coefficient Calibration

`blur` runs the calibration of the blur proportionality coefficient `kappa` linking the spread parameter of the PSF with the blur radius. It updates the internal parameters with the optimized value of `kappa`.

**Requirements**: internal parameters, features and images.

**Output:** internal parameters.

### Inverse Distortion Coefficients Calibration

`invdistortion` runs the calibration of the inverse distortion coefficients `\phi^{-1}` used in the inverse projection model.

**Requirements**: camera parameters, internal parameters and scene configuration.

**Output:** calibrated camera parameters.

  
Datasets
========

* Datasets R12-A, R12-B and R12-C can be downloaded [from here](https://github.com/comsee-research/plenoptic-datasets).
* The dataset R12-D, and the simulated _unfocused plenoptic camera_ dataset UPC-S are also available [from here](https://github.com/comsee-research/plenoptic-datasets).
* Datasets R12-E, ES and ELP20 are available [here](https://github.com/comsee-research/plenoptic-datasets).

Citing
======

If you use COMPOTE or [libpleno] in an academic context, please cite the following publication:

	@inproceedings{labussiere2020blur,
	  title 	=	{Blur Aware Calibration of Multi-Focus Plenoptic Camera},
	  author	=	{Labussi{\`e}re, Mathieu and Teuli{\`e}re, C{\'e}line and Bernardin, Fr{\'e}d{\'e}ric and Ait-Aider, Omar},
	  booktitle	=	{Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
	  pages		=	{2545--2554},
	  year		=	{2020}
	}
	
or 

	@article{labussiere2022calibration
	  title		=	{Leveraging blur information for plenoptic camera calibration},
	  author	=	{Labussi{\`{e}}re, Mathieu and Teuli{\`{e}}re, C{\'{e}}line and Bernardin, Fr{\'{e}}d{\'{e}}ric and Ait-Aider, Omar},
	  doi		=	{10.1007/s11263-022-01582-z},
	  journal	=	{International Journal of Computer Vision},
	  year		=	{2022},
	  month		=	{may},
	  number	=	{2012},
	  pages		=	{1--23}
	}

License
=======

COMPOTE is licensed under the GNU General Public License v3.0. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libv]: http://gitlab.ip.uca.fr/libv/libv
[lma]: http://gitlab.ip.uca.fr/libv/lma
[OpenCV]: https://opencv.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[boost]: http://www.boost.org/
[libpleno]: https://github.com/comsee-research/libpleno

---
