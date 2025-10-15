#include "types/BallInfo.hpp"
#include "utils/debug/Assert.hpp"

const char *const BallHint::TypeName[] =
{
   "bLeft",
   "bRight",
   "bHidden",
   "bNone"
};

void BallInfo::verify() const
{
  	if(status == seen || status == guessed) {
		ASSERT(std::isfinite(imageCoords.x()));
		ASSERT(std::isfinite(imageCoords.y()));
		ASSERT(std::isfinite(rr.vec.x()));
		ASSERT(std::isfinite(rr.vec.y()));
		ASSERT(std::isfinite(rr.vec.z()));
		ASSERT(radius > 0);
  	}
}

// TODO - implement reset (Commented out needs resetting)
void BallInfo::reset() {
   	status = notSeen;  
   	// Matrix2f covarianceOnField;      
   	rr.vec.setZero();
	rr.var.setZero();
   	radius = 0;
	imageCoords = Vector2i::Zero();   
	topCamera = false;
}
