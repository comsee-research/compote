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
#include <pleno/io/printer.h>
#include <pleno/io/choice.h>

//calibration
#include <pleno/processing/calibration/calibration.h>

//config
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera calibration =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Camera information configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("1) Load Camera information from configuration file");
	PlenopticCamera mfpc;
	load(config.path.camera, mfpc);
	
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));
	mfpc.params() = params;

	PRINT_INFO("Camera = " << mfpc << std::endl);
	PRINT_INFO("Internal Parameters = " << params << std::endl);
	
////////////////////////////////////////////////////////////////////////////////
// 2) Load Camera information configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("2) Load Scene information from configuration file");	
	PRINT_WARN("\t2.1) Loading Scene Model");
	SceneConfig cfg_scene;
	v::load(config.path.scene, cfg_scene);
	DEBUG_ASSERT(
		(cfg_scene.checkerboards().size() > 0u),
		"No model available while loading scene"
	);
	
	PRINT_WARN("\t2.1) Loading poses");
	CalibrationPosesConfig cfg_poses;
	v::load(config.path.extrinsics, cfg_poses);
	DEBUG_ASSERT(
		(cfg_poses.poses().size() > 0u),
		"No poses available while loading scene"
	);
	
	CheckerBoards scene; 		
	scene.reserve(cfg_poses.poses().size());
	for (const auto& cfg_pose : cfg_poses.poses())
	{
		cfg_scene.checkerboards()[0].pose() = cfg_pose.pose();
		scene.emplace_back(cfg_scene.checkerboards()[0]);		
	}
	
	for (const auto& cb : scene) 
	{
		PRINT_DEBUG("CheckerBoard at pose : " << cb.pose());
	}
////////////////////////////////////////////////////////////////////////////////
// 3) Starting Calibration of the  inverse distortions
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("4) Starting Calibration of the inverse distortions");
	Distortions invdistortions;
	calibration_inverseDistortions(invdistortions, mfpc, scene);
	
	if(save())
	{
		mfpc.main_lens_invdistortions() = invdistortions;
		
		PRINT_WARN("\t... Saving Intrinsic Parameters");
		save(config.path.output, mfpc);
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

