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

//geometry
#include <pleno/geometry/camera/plenoptic.h>
#include <pleno/geometry/observation.h>

//detection & calibration
#include <pleno/processing/detection/detection.h>

//tools
#include <pleno/processing/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
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
	
	fs::create_directories("obs");

////////////////////////////////////////////////////////////////////////////////
// 1) Load Images from configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("1) Load Images from configuration file");
	ImagesConfig cfg_images;
	v::load(config.path.images, cfg_images);
	
	//1.1) Load whites images
    PRINT_WARN("\t1.1) Load whites images");
	std::vector<ImageWithInfo> whites;	
	load(cfg_images.whites(), whites);
	
	DEBUG_ASSERT((whites.size() != 0u),	"You need to provide white images!");
	
	//1.2) Load checkerboard images
	PRINT_WARN("\t1.2) Load checkerboard images");	
	std::vector<ImageWithInfo> checkerboards;	
	load(cfg_images.checkerboards(), checkerboards);
	
	DEBUG_ASSERT((checkerboards.size() != 0u),	"You need to provide checkerboard images!");
	
	const double cbfnbr = checkerboards[0].fnumber;	
	for (const auto& [ _ , fnumber, __ ] : checkerboards)
	{
		DEBUG_ASSERT((cbfnbr == fnumber), "All checkerboard images should have the same aperture configuration");
	}
	
	//1.3) Load white image corresponding to the aperture (mask)
	PRINT_WARN("\t1.3) Load white image corresponding to the aperture (mask)");
	const auto [mask, mfnbr, __] = ImageWithInfo{ 
				cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
				cfg_images.mask().fnumber()
			};
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
	std::size_t f = 0;
	for (const auto& [ img, _, frame ] : checkerboards)
	{		
		PRINT_INFO("=== Devignetting image frame f = " << f);
		Image unvignetted;	devignetting(img, mask, unvignetted);
			
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
			v::save("obs/bap-observations-"+std::to_string(getpid())+"-frame-0-to-"+std::to_string(f)+".bin.gz", cfg_obs);
		}	
		
		++f;
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

