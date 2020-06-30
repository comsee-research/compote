#include "blur.h"

#include <pleno/io/printer.h>

bool RelativeBlurCostError::operator()(
    const BlurProportionalityCoefficient& kappa,
	ErrorType& error
) const
{    
	constexpr unsigned int W = 7u;
    error.setZero();
	
	Image blured, fedi, fref;
	ref.convertTo(fref, CV_32FC1, 1./255.0); 
	edi.convertTo(fedi, CV_32FC1, 1./255.0);
	
	//compute equally-defocused image
	const double sigma_r = kappa.kappa * rho_r;
	cv::GaussianBlur(fedi, blured, cv::Size{0,0}, sigma_r, sigma_r);	
	
	const double cost = cv::norm(fref, blured, cv::NORM_L1) / (fref.cols * fref.rows);	
	
	error[0] = cost;

#if 0
	cv::namedWindow("ref", cv::WINDOW_NORMAL);
	cv::namedWindow("edi", cv::WINDOW_NORMAL);
	cv::namedWindow("bli", cv::WINDOW_NORMAL);
	
	cv::imshow("ref", fref);
	cv::imshow("edi", fedi);
	cv::imshow("bli", blured);
	
	cv::resizeWindow("ref", 200u, 200u);
	cv::resizeWindow("edi", 200u, 200u);
	cv::resizeWindow("bli", 200u, 200u);
	
	//DEBUG_VAR(cost);
	std::getchar();
#endif
	
    return true;
}
