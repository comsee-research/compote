//STD
#include <iostream>
#include <unistd.h>

//EIGEN

//BOOST

//OPENCV
#include <opencv2/opencv.hpp>
//LIBV
#include <libv/core/conversions/opencv.hpp>

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
#include <pleno/processing/imgproc/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"


int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera poses statistics =========");
	Config_t config = parse_args(argc, argv);
	
	Printer::verbose(config.verbose);
	Printer::level(config.level);
	
	CalibrationPoses poses;
	PRINT_WARN("Loading Calibration Poses");
	CalibrationPosesConfig cfg_poses;
	
	v::load(config.path.extrinsics, cfg_poses);
			
	poses.reserve(cfg_poses.poses().size());
	for(const auto& cfg_pose : cfg_poses.poses()) 
	{
		poses.emplace_back(
			CalibrationPose{ cfg_pose.pose(), cfg_pose.frame() }
		);
	}		
	
    // Sorting poses according to frame index
    std::sort(
    	begin(poses), end(poses),
		[](const auto& p1, const auto& p2) { 
			return p1.frame < p2.frame; 
		}
	);
	
	PRINT_INFO("Enter Ground Truth Displacement : ");
	double dz_gt = 0.;
	std::cin >> dz_gt;
	
	{//ABSOLUTE
		std::vector<double> dists;
		std::transform(
			poses.begin(), poses.end()-1,
			poses.begin()+1, 
			std::back_inserter(dists),
		    [](const auto& p1, const auto& p2) -> double { 
		    	return std::fabs(p1.pose.translation()[2] - p2.pose.translation()[2]); 
		    }
		);
		
		PRINT_INFO("Delta_Z = ");
		std::ostringstream oss;
		for(auto&d : dists) oss << d << " mm" << std::endl;
		PRINT_INFO(oss.str());
		
		double sum = std::accumulate(dists.begin(), dists.end(), 0.0);
		double mean = sum / dists.size();
		
		PRINT_INFO("Mean Dz = " << mean);

		std::vector<double> diff(dists.size());
		std::transform(
			dists.begin(), dists.end(), diff.begin(),
			[&mean](double x) { return x - mean; }
		);
		double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
		double stdev = std::sqrt(sq_sum / dists.size());
		
		PRINT_INFO("Stdev Dz = " << stdev);
	}
	{//RELATIVE
		std::vector<double> dists;
		std::transform(
			poses.begin(), poses.end()-1,
			poses.begin()+1, 
			std::back_inserter(dists),
		    [&dz_gt](const auto& p1, const auto& p2) -> double { 
		    	return 100. * std::fabs(dz_gt - std::fabs(p1.pose.translation()[2] - p2.pose.translation()[2])) / dz_gt; 
		    }
		);
		
		PRINT_INFO("Delta_Z = ");
		std::ostringstream oss;
		for(auto&d : dists) oss << d << "%" << std::endl;
		PRINT_INFO(oss.str());
		
		double sum = std::accumulate(dists.begin(), dists.end(), 0.0);
		double mean = sum / dists.size();
		
		PRINT_INFO("Mean Dz = " << mean);

		std::vector<double> diff(dists.size());
		std::transform(
			dists.begin(), dists.end(), diff.begin(),
			[&mean](double x) { return x - mean; }
		);
		double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
		double stdev = std::sqrt(sq_sum / dists.size());
		
		PRINT_INFO("Stdev Dz = " << stdev);
	}

	PRINT_INFO("========= EOF =========");
	return 0;
}
