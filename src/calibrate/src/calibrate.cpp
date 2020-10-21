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
#include <pleno/processing/preprocess.h> 
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
		PRINT_DEBUG("Load image " << cfg.path());
		images.emplace_back(
			ImageWithInfo{ 
				cv::imread(cfg.path(), cv::IMREAD_UNCHANGED),
				cfg.fnumber(),
				cfg.frame()
			}
		);	
	}
}

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera calibration =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Images from configuration file
////////////////////////////////////////////////////////////////////////////////
	std::vector<ImageWithInfo> whites, checkerboards;
	Image mask;
	
	if (config.path.images == "" and not(config.path.features == ""))
	{
		PRINT_WARN("1) No images loaded");
	}
	else
	{
		PRINT_WARN("1) Load Images from configuration file");
		ImagesConfig cfg_images;
		v::load(config.path.images, cfg_images);
		
		//1.1) Load whites images
		PRINT_WARN("\t1.1) Load whites images");
		//std::vector<ImageWithInfo> whites;	
		load(cfg_images.whites(), whites);
		
		DEBUG_ASSERT((whites.size() != 0u),	"You need to provide white images!");
		
		//1.2) Load checkerboard images
		PRINT_WARN("\t1.2) Load checkerboard images");	
		//std::vector<ImageWithInfo> checkerboards;	
		load(cfg_images.checkerboards(), checkerboards);
		
		DEBUG_ASSERT((checkerboards.size() != 0u),	"You need to provide checkerboard images!");
		
		const double cbfnbr = checkerboards[0].fnumber;	
		for (const auto& [ _ , fnumber, __ ] : checkerboards)
		{
			DEBUG_ASSERT((cbfnbr == fnumber), "All checkerboard images should have the same aperture configuration");
		}
		
		//1.3) Load white image corresponding to the aperture (mask)
		PRINT_WARN("\t1.3) Load white image corresponding to the aperture (mask)");
		const auto [mask_, mfnbr, __ ] = ImageWithInfo{ 
					cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
					cfg_images.mask().fnumber()
				};
		mask = mask_;
		DEBUG_ASSERT((mfnbr == cbfnbr), "No corresponding f-number between mask and images");
	}
////////////////////////////////////////////////////////////////////////////////
// 2) Load Camera information from configuration file
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("2) Load Camera information from configuration file");
	PlenopticCameraConfig cfg_camera;
	v::load(config.path.camera, cfg_camera);

	//2.1) Sensor parameters intialization
    PRINT_WARN("\t2.1) Sensor parameters intialization");
	Sensor sensor{cfg_camera.sensor()};
	
	//2.2) Grid parameters initialization
	PRINT_WARN("\t2.2) MIA geometry parameters initialization");
    MIA mia{cfg_camera.mia()};
    
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).pen_color(v::purple).pen_width(5).name("main:initialgrid(purple)"), mia);
    
////////////////////////////////////////////////////////////////////////////////
// 3) Pre-calibration step
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("3) Pre-calibration");
	InternalParameters params;
	if(config.path.params == "") //no params available
	{
		PRINT_WARN("3) Pre-calibration: MIA geometry calibration");
		//3.1) Compute micro-image centers
		PRINT_WARN("\t3.1) Compute micro-image centers");
		MICObservations mic_obs;
		
		for(const auto& [img, fnumber, __] : whites)
		{
			if(fnumber <= 4.) continue; //micro-images are overlapping
			PRINT_INFO("=== Computing MIC in image f/" << fnumber);
			MICObservations obs = detection_mic(img);
			mic_obs.insert(std::end(mic_obs), std::begin(obs), std::end(obs));
		
			GUI(
				RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("main:white_image_f/"+std::to_string(fnumber)), img);
				for (const auto& o : obs)
					RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("main:observations(blue)").pen_color(v::blue).pen_width(5), P2D{o[0], o[1]});    
				Viewer::update();
			);
		}	
		//3.2) Optimization
		PRINT_WARN("\t3.2) MIA geometry parameters calibration");
		calibration_MIA(mia, mic_obs);
	   
		PRINT_DEBUG("Optimized MIA geometry parameters = \n" << mia);
		RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).pen_color(v::green).pen_width(5).name("main:optimizedgrid(green)"), mia);
		clear();	

		PRINT_WARN("3) Pre-calibration: Preprocessing white images and Computing internal parameters");
		FORCE_GUI(true);
		params = preprocess(whites, mia, sensor.scale(), cfg_camera.I(), cfg_camera.mode());
		FORCE_GUI(false);
		v::save("params-"+std::to_string(getpid())+".js", v::make_serializable(&params));
	}
	else
	{
		PRINT_WARN("3) Load internal parameters from configuration file");
		v::load(config.path.params, v::make_serializable(&params));
	}	
	PRINT_INFO("Internal Parameters = " << params << std::endl);
	clear();

////////////////////////////////////////////////////////////////////////////////
// 4) Features extraction step
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("4) Features extraction");	
	BAPObservations bap_obs;
	MICObservations center_obs;	
	
	if(config.path.features == "") //no features available
	{
		//4.1) For each frame detect corners
		PRINT_WARN("\t4.1) Computing BAP Features");
		std::size_t f = 0;
		for (const auto& [ img, _, frame ] : checkerboards)
		{					
			PRINT_INFO("=== Devignetting image frame f = " << f);
			Image unvignetted;
			devignetting(img, mask, unvignetted);
				
			PRINT_INFO("=== Detecting BAP Observation in image frame f = " << f);
			BAPObservations bapf = detection_bapfeatures(unvignetted, mia, params);
			
			//assign frame index
			std::for_each(
				bapf.begin(), bapf.end(), 
				[&f](BAPObservation& cbo) {
					cbo.frame = f;
				}
			);
							
			bap_obs.insert(std::end(bap_obs), 
				std::make_move_iterator(std::begin(bapf)),
				std::make_move_iterator(std::end(bapf))
			);

			++f;
			PRINT_INFO(std::endl);
			clear();
		}	
		//4.2) Computing MIC Features
		PRINT_WARN("\t4.2) Computing MIC Features");
		center_obs = detection_mic(whites[1].img);
		
		//4.3) Saving Features
		PRINT_WARN("\t4.3) Saving Features");
		if(save())
		{
			ObservationsConfig cfg_obs;
			cfg_obs.features() = bap_obs;
			cfg_obs.centers() = center_obs;			
			v::save("observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
		}
	}
	else // features available
	{	
		//5.3) Loading Features
		PRINT_WARN("\t... Loading Features");
		ObservationsConfig cfg_obs;
		v::load(config.path.features, cfg_obs);

		bap_obs = cfg_obs.features(); DEBUG_VAR(bap_obs.size());
		center_obs = cfg_obs.centers(); DEBUG_VAR(center_obs.size());
		
		DEBUG_ASSERT(
			((bap_obs.size() > 0u) and (center_obs.size() > 0u)), 
			"No observations available (missing features or centers)"
		);
	}	

////////////////////////////////////////////////////////////////////////////////
// 5) Starting Calibration of the MFPC
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("5) Starting Calibration of the Plenoptic Camera");
	//5.1) Loading Scene Model
	PRINT_WARN("\t5.1) Loading Scene Model");
	SceneConfig cfg_scene;
	v::load(config.path.scene, cfg_scene);
	DEBUG_ASSERT(
		(cfg_scene.checkerboards().size() > 0u),
		"No model available while loading scene"
	);
	
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
	
	PRINT_WARN("\t5.2) Computing Initial Model");
	PlenopticCamera mfpc;
	{
		const double F = cfg_camera.main_lens().f();
		const double N = cfg_camera.main_lens().aperture();
		const double h = cfg_camera.dist_focus();
		const PlenopticCamera::Mode mode = (cfg_camera.mode() != -1) ? PlenopticCamera::Mode(cfg_camera.mode()) : PlenopticCamera::Mode::Galilean;
		
		mfpc.init(sensor, mia, params, F, N, h, mode);
	}
	PRINT_INFO("=== Initial Camera Parameters " << std::endl << "MFPC = " << mfpc);

	PRINT_WARN("\t5.3) Calibrate");	
	CalibrationPoses poses;
	calibration_PlenopticCamera(poses, mfpc, scene, bap_obs, center_obs, pictures);

	PRINT_WARN("\t5.4) Save Calibration Parameters");
	if(save()) 
	{
		PRINT_WARN("\t... Saving Intrinsic Parameters");
		save(config.path.output, mfpc);
		
		PRINT_WARN("\t... Saving Extrinsics Parameters");
		CalibrationPosesConfig cfg_poses;
		cfg_poses.poses().resize(poses.size());
		
		int i=0;
		for(const auto& [p, f] : poses) {
			DEBUG_VAR(f); DEBUG_VAR(p); 
			cfg_poses.poses()[i].pose() = p;
			cfg_poses.poses()[i].frame() = f;
			++i;
		}
		
		v::save(config.path.extrinsics, cfg_poses);
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

