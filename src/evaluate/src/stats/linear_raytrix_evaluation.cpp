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

#include <pleno/io/printer.h>

//geometry
#include <pleno/geometry/observation.h>

//tools
#include <pleno/processing/improcess.h> //devignetting

//config
#include <pleno/io/cfg/images.h>
#include <pleno/io/cfg/camera.h>
#include <pleno/io/cfg/scene.h>
#include <pleno/io/cfg/observations.h>
#include <pleno/io/cfg/poses.h>

#include "utils.h"

struct xyz {
	double x,y,z;
};

std::vector<xyz> read_xyz(std::string path)
{
	static int n =0;
	std::ifstream ifs(path);
	PRINT_ERR("Open file = " << path);
    int count;
    ifs >> count;
    ifs.ignore(1, '\n');
    ifs.ignore(1, '\n');
    DEBUG_VAR(count);
    
    std::vector<xyz> pts; pts.reserve(count);
    
    for(int i = 0; i < count; i++)
    {
        std::string line;
        if(!std::getline(ifs, line)) {PRINT_ERR("("<<i << " - " <<n<<") no line"); break; }
        
        //DEBUG_VAR(line);

        std::istringstream iss(line);
		
		xyz p;

        iss >> p.x >> p.y >> p.z;
        
        //DEBUG_VAR(p.x);DEBUG_VAR(p.y);DEBUG_VAR(p.z);
        pts.emplace_back(p);    
        iss.clear(); 
    }	
    
    ifs.close();
	n++;
	return pts;
}


int main(int argc, char* argv[])
{
	PRINT_INFO("========= Multifocus plenoptic camera poses statistics on Raytrix data =========");
	Config_t config = parse_args(argc, argv);
	
	Printer::verbose(config.verbose);
	Printer::level(config.level);
	
	std::vector<std::vector<xyz>> poses_xyz;
	PRINT_WARN("Loading Calibration Poses");
	ImagesConfig cfg_xyz;
	
	v::load(config.path.extrinsics, cfg_xyz);
			
	poses_xyz.reserve(cfg_xyz.checkerboards().size());
	
	for(auto& cfg : cfg_xyz.checkerboards())
	{
		poses_xyz.emplace_back(
			read_xyz(cfg.path())
		);	
	}
	
	//reduce	
	int i=0;
	std::vector<double> poses; poses.reserve(poses_xyz.size());
	std::transform(
		poses_xyz.begin(), poses_xyz.end(),
		std::back_inserter(poses),
	    [&i](const auto& xyz) -> double { 
	    	std::vector<double> zs; zs.reserve(xyz.size());
	    	std::transform(
				xyz.begin(), xyz.end(),
				std::back_inserter(zs),
				[](const auto&p) -> double { return p.z; }
			);
		
	    	std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
	    	
	    	PRINT_ERR("Pose ("<<i++<<") = "<< zs[zs.size() / 2]);
			return zs[zs.size() / 2];
	    
	    	double sum = std::accumulate(
	    		xyz.begin(), xyz.end(), 0.0,
	    		[](double acc, const auto& pt) -> double {
	    			return acc + pt.z;
	    		}
	    	);
	    	PRINT_ERR("Pose ("<<i++<<") = "<< sum / xyz.size());
			return sum / xyz.size();
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
				return std::fabs(p1- p2); 
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
		    	return 100. * std::fabs(dz_gt - std::fabs(p1 - p2)) / dz_gt; 
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
