//STD
#include <iostream>
#include <unistd.h>

#include <experimental/filesystem> //if gcc < 8
namespace fs = std::experimental::filesystem;

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

//tools
#include <pleno/processing/imgproc/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/observations.h>

#include <pleno/io/images.h>

#include "utils.h"

int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera calibration =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());
	
	fs::create_directories("obs");

////////////////////////////////////////////////////////////////////////////////
// 1) Load Images from configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("1) Load Images from configuration file");
	ImagesConfig cfg_images;
	v::load(config.path.images, cfg_images);
	DEBUG_ASSERT((cfg_images.meta().rgb()), "Images must be in rgb format.");
	DEBUG_ASSERT((cfg_images.meta().format() <= 16), "Floating-point images not supported.");
	
	//1.1) Load whites images
    PRINT_WARN("\t1.1) Load whites images");
	std::vector<ImageWithInfo> whites;	
	load(cfg_images.whites(), whites, cfg_images.meta().debayer());
	
	DEBUG_ASSERT((whites.size() != 0u),	"You need to provide white images!");
	
	//1.2) Load checkerboard images
	PRINT_WARN("\t1.2) Load checkerboard images");	
	std::vector<ImageWithInfo> checkerboards;	
	load(cfg_images.checkerboards(), checkerboards, cfg_images.meta().debayer());
	
	DEBUG_ASSERT((checkerboards.size() != 0u),	"You need to provide checkerboard images!");
	
	const double cbfnbr = checkerboards[0].fnumber;	
	for (const auto& [ _ , fnumber, __ ] : checkerboards)
	{
		DEBUG_ASSERT((cbfnbr == fnumber), "All checkerboard images should have the same aperture configuration");
	}
	
	//1.3) Load white image corresponding to the aperture (mask)
	PRINT_WARN("\t1.3) Load white image corresponding to the aperture (mask)");
	ImageWithInfo mask_;
	load(cfg_images.mask(), mask_, cfg_images.meta().debayer());
	
	const auto [mask, mfnbr, __] = mask_;
	DEBUG_ASSERT((mfnbr == cbfnbr), "No corresponding f-number between mask and images");
	
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
    
	PRINT_WARN("\t2.3) Internal parameters initialization");
	InternalParameters params;
	{
		v::load(config.path.params, v::make_serializable(&params));
	}	
	PRINT_INFO("Internal Parameters = " << params << std::endl);

////////////////////////////////////////////////////////////////////////////////
// 3) Detect Features
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("3) Detect features in images");
	BAPObservations bap_obs;
	MICObservations center_obs;	

	PRINT_WARN("\t3.1) Computing BAP Features");
	std::size_t f_ = 0;
	for (const auto& [ img, _, frame ] : checkerboards)
	{		
		std::size_t f = (frame != -1) ? frame : f_;
		
		PRINT_INFO("=== Devignetting image frame f = " << f);
		Image unvignetted;	
		if (cfg_images.meta().format() == 8u) devignetting(img, mask, unvignetted);
		else /* if (cfg_images.meta().format() == 16u) */ devignetting_u16(img, mask, unvignetted);
			
		PRINT_INFO("=== Detecting BAP Observation in image frame f = " << f);
		BAPObservations bapf = detection_bapfeatures(unvignetted, mia, params);
		
		//assign frame index
		std::for_each(bapf.begin(), bapf.end(), [&f](BAPObservation& cbo) { cbo.frame = f; });
		
		//save observation of the current frame	
		{
			ObservationsConfig cfg_obs;
			cfg_obs.features() = bapf;
			v::save("obs/bap-observations-"+std::to_string(getpid())+"-frame-"+std::to_string(f)+".bin.gz", cfg_obs);
		}
			
		//update observations				
		bap_obs.insert(std::end(bap_obs), 
			std::make_move_iterator(std::begin(bapf)),
			std::make_move_iterator(std::end(bapf))
		);
			
	 	//save current cummulated observations		
		{
			ObservationsConfig cfg_obs;
			cfg_obs.features() = bap_obs;
			v::save("obs/bap-observations-"+std::to_string(getpid())+"-frame-x-to-"+std::to_string(f)+".bin.gz", cfg_obs);
		}	
		
		++f_;
		PRINT_INFO(std::endl);
		clear();
	}	
	//5.4) Computing MIC Features
	PRINT_WARN("\t3.2) Computing MIC Features");
	center_obs = detection_mic(whites[1].img);
		
	//save centers observations
	{
		ObservationsConfig cfg_obs;
		cfg_obs.centers() = center_obs;
		v::save("obs/centers-observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
	}
		
	//5.5) Saving Features
	PRINT_WARN("\t3.3) Saving Features");
	ObservationsConfig cfg_obs;
	cfg_obs.features() = bap_obs;
	cfg_obs.centers() = center_obs;			
	v::save("observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

