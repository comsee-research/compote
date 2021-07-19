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
#include <pleno/processing/precalibration/preprocess.h> 
#include <pleno/processing/detection/detection.h>
#include <pleno/processing/calibration/calibration.h>

//tools
#include <pleno/processing/imgproc/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>

#include <pleno/io/images.h>

#include "utils.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera pre-calibration =========");
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
	DEBUG_ASSERT((cfg_images.meta().rgb()), "Images must be in rgb format.");
	DEBUG_ASSERT((cfg_images.meta().format() <= 16), "Floating-point images not supported.");

	std::vector<ImageWithInfo> whites;	
	load(cfg_images.whites(), whites, cfg_images.meta().debayered());
	
	DEBUG_ASSERT((whites.size() != 0u), "You need to provide white images if no features are given !");
	
////////////////////////////////////////////////////////////////////////////////
// 2) Load Camera information from configuration file
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("2) Load Camera information from configuration file");
	PlenopticCameraConfig cfg_camera;
	v::load(config.path.camera, cfg_camera);

	//2.1) Sensor parameters intialization
    PRINT_WARN("\t2.1) Sensor parameters intialization");
	Sensor sensor{cfg_camera.sensor()};
	DEBUG_VAR(sensor);
	
	//2.2) Grid parameters initialization
	PRINT_WARN("\t2.2) MIA geometry parameters initialization");
    MIA mia{cfg_camera.mia()};
    DEBUG_VAR(mia);
        
////////////////////////////////////////////////////////////////////////////////
// 3) Grid Parameters calibration
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("3) MIA geometry Parameters calibration");
	//3.1) Compute micro-image centers
	PRINT_WARN("\t3.1) Compute micro-image centers");
	MICObservations mic_obs;
	
	for(const auto& [img, fnumber, __] : whites)
	{
		if(fnumber <= cfg_camera.main_lens().aperture() and cfg_camera.mode() != PlenopticCamera::Mode::Unfocused) continue; //micro-images are overlapping
		
		PRINT_INFO("=== Computing MIC in image f/" << fnumber);
		MICObservations obs = detection_mic(img, cfg_camera.I());
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
   
    PRINT_INFO("Optimized MIA geometry parameters = \n" << mia);  
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).pen_color(v::green).pen_width(5).name("main:optimizedgrid(green)"), mia);
	clear();
	
	wait();
////////////////////////////////////////////////////////////////////////////////
// 4) Preprocess white images and Set internal parameters
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("4) Preprocessing white images and Computing internal parameters");
	InternalParameters params;

FORCE_GUI(true);
	params = preprocess(whites, mia, sensor.scale(), cfg_camera.I(), cfg_camera.mode(), cfg_camera.main_lens().aperture());
FORCE_GUI(false);

	PRINT_INFO("Internal Parameters = " << params << std::endl);
	clear();
		
	if(save())
	{
		PRINT_WARN("5) Saving camera parameters");
		PlenopticCamera mfpc;
		{
			const double F = cfg_camera.main_lens().f();
			const double N = cfg_camera.main_lens().aperture();
			const double h = cfg_camera.dist_focus();
			const PlenopticCamera::Mode mode = (cfg_camera.mode() != -1) ? PlenopticCamera::Mode(cfg_camera.mode()) : PlenopticCamera::Mode::Galilean;
			
			mfpc.init(sensor, mia, params, F, N, h, mode);
		}
		save("camera-"+std::to_string(getpid())+".js", mfpc);
		v::save(config.path.params, v::make_serializable(&(mfpc.params())));
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

