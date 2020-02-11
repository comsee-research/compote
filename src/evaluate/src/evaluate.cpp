//STD
#include <iostream>
#include <unistd.h>
//EIGEN
//BOOST
//OPENCV
#include <opencv2/opencv.hpp>

//LIBPLENO
#include <pleno/types.h>

#include <pleno/graphic/gui.h>
#include <pleno/graphic/viewer_2d.h>
#include <pleno/io/printer.h>

//geometry
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

//detection & calibration
#include <pleno/processing/detection/detection.h>
#include <pleno/processing/calibration/calibration.h>

//tools
#include <pleno/processing/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"

void clear() {
	GUI(
		PRINT_WARN("Clear viewer ? [y/n]");
		char c;
		std::cin >> c;
		if(c == 'y') {Viewer::clear(); PRINT_DEBUG("Cleared !"); }	
		std::cin.clear();
		while (std::cin.get() != '\n');
	);
}

bool save() {
	bool ret = false;
	if(Printer::level() bitand Printer::Level::WARN)
	{
		PRINT_WARN("Save ? [y/n]");
		char c;
		std::cin >> c;
		if(c == 'y') { ret = true; }	
		std::cin.clear();
		while (std::cin.get() != '\n');
	}
	return ret;
}

void load(const std::vector<ImageWithInfoConfig>& cfgs, std::vector<ImageWithInfo>& images)
{
	images.reserve(cfgs.size());
	
	for(const auto& cfg : cfgs)
	{
		images.emplace_back(
			ImageWithInfo{ 
				cv::imread(cfg.path(), cv::IMREAD_UNCHANGED),
				cfg.fnumber()
			}
		);	
	}
}

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera evaluation =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui);
	
	Printer::verbose(config.verbose);
	Printer::level(config.level);

//1) Load Camera information from configuration file
	PRINT_WARN("1) Load Camera information from configuration file");
	PlenopticCamera mfpc;
	load(config.path.camera, mfpc);
	
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));
	mfpc.params() = params;
	
//2) Load images from configuration file
	PRINT_WARN("2) Load images from configuration file");
	PRINT_WARN("\t2.1) Load white image corresponding to the aperture mask");
	ImagesConfig cfg_images;
	v::load(config.path.images, cfg_images);
	
	const auto [mask, mfnbr] = ImageWithInfo{ 
				cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
				cfg_images.mask().fnumber()
			};
	
	PRINT_WARN("\t2.2) Load checkerboard images");
	std::vector<ImageWithInfo> checkerboards;	
	load(cfg_images.checkerboards(), checkerboards);
	
//3) Loading Features
	PRINT_WARN("3) Loading BAP Features");
	ObservationsConfig cfg_obs;
	v::load(config.path.features, cfg_obs);
		
	BAPObservations bap_obs = cfg_obs.features();
	
//4) Starting Evaluation of the MutliFocus Plenoptic Camera Calibration
	PRINT_WARN("4) Starting Evaluation of the MutliFocus Plenoptic Camera Calibration");
	//4.1) Loading Scene Model
	PRINT_WARN("\t4.1) Loading Scene Model");
	SceneConfig cfg_scene;
	v::load(config.path.scene, cfg_scene);
	DEBUG_ASSERT((cfg_scene.checkerboards().size() > 0u), "No checkerboard model provided.");
	
	CheckerBoard scene{cfg_scene.checkerboards()[0]};
			
	std::vector<Image> pictures;
	pictures.reserve(checkerboards.size());
	
	std::transform(
		checkerboards.begin(), checkerboards.end(),
		std::back_inserter(pictures),
		[&mask](const auto& iwi) -> Image { 
			Image unvignetted;
			devignetting(iwi.img, mask, unvignetted);
    		Image img = Image::zeros(unvignetted.rows, unvignetted.cols, CV_8UC1);
			cv::cvtColor(unvignetted, img, cv::COLOR_BGR2GRAY);
			return img; 
		}	
	);	
	
	PRINT_WARN("\t4.2) Calibrate Extrinsics");
	CalibrationPoses poses;
	calibration_ExtrinsicsPlenopticCamera(poses, mfpc, scene, bap_obs, pictures);
	
	PRINT_WARN("\t6.3) Save Extrinsics Poses");
	if(save()) 
	{
		CalibrationPosesConfig cfg_poses;
		cfg_poses.poses().resize(poses.size());
		
		int i=0;
		for(const auto& [p, f] : poses) {
			DEBUG_VAR(f); DEBUG_VAR(p); 
			cfg_poses.poses()[i].pose() = p;
			cfg_poses.poses()[i].frame() = f;
			++i;
		}
		
		v::save( config.path.extrinsics, cfg_poses );
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

