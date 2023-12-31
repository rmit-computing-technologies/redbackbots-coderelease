#pragma once

#include "types/Point.hpp"
#include "types/BBox.hpp"

struct FootInfo
{
   FootInfo (){}
   FootInfo (BBox& _a, BBox& _b, int _age):robotBounds(_a), imageBounds(_b), age(_age){}
   bool operator==(const FootInfo &other) const{
	   return 	imageBounds == other.imageBounds
	   	   && 	robotBounds == other.robotBounds
	   	   &&   age 		== other.age;
   }
   BBox robotBounds, imageBounds;
   int age;
};
