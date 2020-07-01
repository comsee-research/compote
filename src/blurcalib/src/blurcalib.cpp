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

//geometry
#include <pleno/geometry/observation.h>

//detection & calibration
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
	PRINT_INFO("========= Multifocus plenoptic camera calibration =========");
	Config_t config = parse_args(argc, argv);
	
	Viewer::enable(config.use_gui); DEBUG_VAR(Viewer::enable());
	
	Printer::verbose(config.verbose); DEBUG_VAR(Printer::verbose());
	Printer::level(config.level); DEBUG_VAR(Printer::level());

////////////////////////////////////////////////////////////////////////////////
// 1) Load Images from configuration file
////////////////////////////////////////////////////////////////////////////////
	std::vector<ImageWithInfo> checkerboards;
	Image mask;
	{
		PRINT_WARN("1) Load Images from configuration file");
		ImagesConfig cfg_images;
		v::load(config.path.images, cfg_images);
	
		//1.2) Load checkerboard images
		PRINT_WARN("\t1.1) Load checkerboard images");	
		//std::vector<ImageWithInfo> checkerboards;	
		load(cfg_images.checkerboards(), checkerboards);
		
		DEBUG_ASSERT((checkerboards.size() != 0u),	"You need to provide checkerboard images!");
		
		const double cbfnbr = checkerboards[0].fnumber;	
		for (const auto& [ _ , fnumber] : checkerboards)
		{
			DEBUG_ASSERT((cbfnbr == fnumber), "All checkerboard images should have the same aperture configuration");
		}
		
		//1.3) Load white image corresponding to the aperture (mask)
		PRINT_WARN("\t1.2) Load white image corresponding to the aperture (mask)");
		const auto [mask_, mfnbr] = ImageWithInfo{ 
					cv::imread(cfg_images.mask().path(), cv::IMREAD_UNCHANGED),
					cfg_images.mask().fnumber()
				};
		mask = mask_;
		DEBUG_ASSERT((mfnbr == cbfnbr), "No corresponding f-number between mask and images");
	}
    
////////////////////////////////////////////////////////////////////////////////
// 2) Load internal parameters from configuration file
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("2) Load internal parameters from configuration file");
	InternalParameters params;
	v::load(config.path.params, v::make_serializable(&params));

	PRINT_INFO("Internal Parameters = " << params << std::endl);

////////////////////////////////////////////////////////////////////////////////
// 3) Features extraction step
////////////////////////////////////////////////////////////////////////////////
	PRINT_WARN("3) Load Features");	
	BAPObservations bap_obs;
	{
		ObservationsConfig cfg_obs;
		v::load(config.path.features, cfg_obs);

		bap_obs = cfg_obs.features(); DEBUG_VAR(bap_obs.size());
		
		DEBUG_ASSERT(
			((bap_obs.size() > 0u)), 
			"No observations available (missing features)"
		);
	}	

////////////////////////////////////////////////////////////////////////////////
// 4) Starting Calibration of the Relative blur Radius
////////////////////////////////////////////////////////////////////////////////	
	PRINT_WARN("4) Starting Calibration of the Relative blur Radius");
	PRINT_WARN("\t4.1) Devignetting images");
			
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

	PRINT_WARN("\t4.2) Calibrate");	
	calibration_relativeBlur(params, bap_obs, pictures);
	
	if(save())
	{
		PRINT_WARN("5) Saving internals parameters");
		v::save("params-"+std::to_string(getpid())+".js", v::make_serializable(&params));
	}
	
	PRINT_INFO("========= EOF =========");

	Viewer::wait();
	Viewer::stop();
	return 0;
}

