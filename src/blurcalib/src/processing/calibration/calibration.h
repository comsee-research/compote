#pragma once

#include <pleno/types.h>
#include <pleno/geometry/internals.h>
#include <pleno/geometry/observation.h>

void calibration_relativeBlur(
	const InternalParameters& internals,     
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
);
