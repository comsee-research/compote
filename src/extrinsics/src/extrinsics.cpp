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
#include <pleno/io/choice.h>

//geometry
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

//detection & calibration
#include <pleno/processing/detection/detection.h>
#include <pleno/processing/calibration/calibration.h>

//tools
#include <pleno/processing/imgproc/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include <pleno/io/images.h>

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera extrinsics evaluation =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui);
	
	Printer::verbose(config.verbose);
	Printer::level(config.level);
////////////////////////////////////////////////////////////////////////////////	
// 1) Load Camera information from configuration file
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("1) Load Camera information from configuration file");
	PlenopticCamera mfpc;
	load(config.path.camera, mfpc);
	
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));
	mfpc.params() = params;
	
////////////////////////////////////////////////////////////////////////////////		
// 2) Load images from configuration file
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("2) Load images from configuration file");
	ImagesConfig cfg_images;
	v::load(config.path.images, cfg_images);
	DEBUG_ASSERT((cfg_images.meta().rgb()), "Images must be in rgb format.");
	DEBUG_ASSERT((cfg_images.meta().format() < 16), "Floating-point images not supported.");
	
	PRINT_WARN("\t2.1) Load white image corresponding to the aperture mask");
	ImageWithInfo mask_;
	load(cfg_images.mask(), mask_, cfg_images.meta().debayer());
	
	const auto [mask, mfnbr, __] = mask_;
	
	PRINT_WARN("\t2.2) Load checkerboard images");
	std::vector<ImageWithInfo> checkerboards;	
	load(cfg_images.checkerboards(), checkerboards, cfg_images.meta().debayer());

////////////////////////////////////////////////////////////////////////////////	
// 3) Loading Features
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("3) Loading BAP Features");
	ObservationsConfig cfg_obs;
	v::load(config.path.features, cfg_obs);
		
	BAPObservations bap_obs = cfg_obs.features();

////////////////////////////////////////////////////////////////////////////////	
// 4) Starting Evaluation of the MutliFocus Plenoptic Camera Calibration
////////////////////////////////////////////////////////////////////////////////	
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
		[&mask, format = cfg_images.meta().format()](const auto& iwi) -> Image { 
			Image unvignetted;
			
			if (format == 8u) devignetting(iwi.img, mask, unvignetted);
			else /* if (format == 16u) */ devignetting_u16(iwi.img, mask, unvignetted);
			
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
		
		v::save(config.path.extrinsics, cfg_poses );
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

