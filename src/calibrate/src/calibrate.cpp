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
#include <pleno/geometry/camera/mfpc.h>
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

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera calibration =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load white images from configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("1) Load white images from configuration file");
	ImagesConfig cfg_images;
	v::load(config.path.images, cfg_images);

	std::vector<ImageWithInfo> whites;	
	whites.reserve(cfg_images.whites().size());
	
	for(auto& cfg_image : cfg_images.whites())
	{
		whites.emplace_back(
			ImageWithInfo{ 
				cv::imread(cfg_image.path(), cv::IMREAD_UNCHANGED),
				cfg_image.fnumber()
			}
		);	
	}
	DEBUG_ASSERT(
		((whites.size() != 0u) or ((whites.size() == 0u) and (config.path.features == ""))),
		"You need to provide white images if no features are given !"
	);
	
////////////////////////////////////////////////////////////////////////////////
// 2) Load Camera information from configuration file
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("2) Load Camera information from configuration file");
	MFPCConfig cfg_camera;
	v::load(config.path.camera, cfg_camera);

	//2.1) Sensor parameters intialization
    PRINT_WARN("\t2.1) Sensor parameters intialization");
	Sensor sensor{cfg_camera.sensor()};
	
	//2.2) Grid parameters initialization
	PRINT_WARN("\t2.2) MIA geometry parameters initialization");
    MIA mia{cfg_camera.mia()};

	PRINT_DEBUG("Initial MIA geometry parameters:\n" << mia);
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).pen_color(v::purple).pen_width(5).name("main:initialgrid(purple)"), mia);
    
////////////////////////////////////////////////////////////////////////////////
// 3) Grid Parameters calibration
////////////////////////////////////////////////////////////////////////////////
#if 0 //OPTIMIZATION
	PRINT_WARN("3) MIA geometry Parameters calibration");
	//3.1) Compute micro-image centers
	PRINT_WARN("\t3.1) Compute micro-image centers");
	MICObservations mic_obs;
	
	for(const auto& [img, fnumber] : whites)
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
#else
	PRINT_WARN("3) MIA geometry Parameters calibration (skipped)");
#endif
	clear();	

////////////////////////////////////////////////////////////////////////////////
// 4) Preprocess white images and Set internal parameters
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("4) Preprocessing white images and Computing internal parameters");
	InternalParameters params;
	if(config.path.params == "") //no params available
	{
		params = preprocess(whites, mia, sensor.scale());
		v::save("params-"+std::to_string(getpid())+".js", v::make_serializable(&params));
	}
	else
	{
		v::load(config.path.params, v::make_serializable(&params));
	}	
	PRINT_INFO("Internal Parameters = " << params << std::endl);
	clear();

////////////////////////////////////////////////////////////////////////////////
// 5) Preprocessing checkerboard images
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("5) Preprocessing checkerboard images");
	//5.1) Load checkerboard images
	PRINT_WARN("\t5.1) Load checkerboard images from configuration file");
	
	std::vector<ImageWithInfo> checkerboards;	
	checkerboards.reserve(cfg_images.checkerboards().size());
	
	for(auto& cfg_image : cfg_images.checkerboards())
	{
		checkerboards.emplace_back(
			ImageWithInfo{ 
				cv::imread(cfg_image.path(), cv::IMREAD_UNCHANGED),
				cfg_image.fnumber()
			}
		);	
	}	
	const double cbfnbr = checkerboards[0].fnumber;	
	for (const auto& [ _ , fnumber] : checkerboards)
		DEBUG_ASSERT((cbfnbr == fnumber), "All checkerboard images should have the same aperture configuration");
	
	//5.2) Load white image corresponding to the aperture (mask)
	PRINT_WARN("\t5.2) Load white image corresponding to the aperture mask");
	const auto [mask, mfnbr] = ImageWithInfo{ 
				cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
				cfg_images.mask().fnumber()
			};
	DEBUG_ASSERT((mfnbr == cbfnbr), "No corresponding f-number between mask and images");

	BAPObservations bap_obs;
	MICObservations center_obs;	
	
	if(config.path.features == "") //no features available
	{
		//5.3) For each frame detect corners
		PRINT_WARN("\t5.3) Computing BAP Features");
	
		std::size_t f = 0;
		for (const auto& [ img, _ ] : checkerboards)
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
			
#if 1 //SAVE CURRENT OBSERVATIONS			
{
	ObservationsConfig cfg_obs;
	cfg_obs.features() = bapf;
	v::save("bap-observations-"+std::to_string(getpid())+"-frame-"+std::to_string(f)+".bin.gz", cfg_obs);
}
#endif					
			bap_obs.insert(std::end(bap_obs), 
				std::make_move_iterator(std::begin(bapf)),
				std::make_move_iterator(std::end(bapf))
			);
			
#if 1 //SAVE CURRENT CUMMULATED OBSERVATIONS			
{
	ObservationsConfig cfg_obs;
	cfg_obs.features() = bap_obs;
	v::save("bap-observations-"+std::to_string(getpid())+"-frame-0-to-"+std::to_string(f)+".bin.gz", cfg_obs);
}
#endif		
			++f;
			PRINT_INFO(std::endl);
			clear();
		}

Viewer::enable(false);		
		//5.4) Computing MIC Features
		PRINT_WARN("\t5.4) Computing MIC Features");
		center_obs = detection_mic(whites[1].img);
#if 1 //SAVE CENTERS OBSERVATIONS
{
	ObservationsConfig cfg_obs;
	cfg_obs.centers() = center_obs;
	v::save("centers-observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
}
#endif
		
		//5.5) Saving Features
		PRINT_WARN("\t5.5) Saving Features");
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
		PRINT_WARN("\t5.3) Loading Features");
		ObservationsConfig cfg_obs;
		v::load(config.path.features, cfg_obs);

		bap_obs = cfg_obs.features();
		center_obs = cfg_obs.centers();
		
		DEBUG_ASSERT(
			((bap_obs.size() > 0u) and (center_obs.size() > 0u)), 
			"No observations available (missing features or centers)"
		);
	}	

////////////////////////////////////////////////////////////////////////////////
// 6) Starting Calibration of the MFPC
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("6) Starting Calibration of the MutliFocus Plenoptic Camera");
	//6.1) Loading Scene Model
	PRINT_WARN("\t6.1) Loading Scene Model");
	SceneConfig cfg_scene;
	v::load(config.path.scene, cfg_scene);
	assert(cfg_scene.checkerboards().size() > 0u);
	
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
	
	PRINT_WARN("\t6.2) Computing Initial Model");
	MultiFocusPlenopticCamera mfpc;
	{
		const double F = cfg_camera.main_lens().f();
		const double N = cfg_camera.main_lens().aperture();
		const double h = cfg_camera.dist_focus();
		
		mfpc.init(sensor, mia, params, F, N, h, MultiFocusPlenopticCamera::Mode::Keplerian) ;
	}
	PRINT_INFO("=== Initial Camera Parameters " << std::endl << "MFPC = " << mfpc);

	PRINT_WARN("\t6.3) Calibrate");
	CalibrationPoses poses;
	calibration_MFPC<false>(poses, mfpc, scene, bap_obs, center_obs, pictures);
	
	PRINT_WARN("\t6.4) Save Calibration Parameters");
	DEBUG_VAR(mfpc);
	if(save()) 
	{
		save(config.path.output, mfpc);
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

