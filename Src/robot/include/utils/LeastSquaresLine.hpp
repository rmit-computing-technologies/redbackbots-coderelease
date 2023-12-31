#pragma once

#include "types/Point.hpp"

class LeastSquaresLine
{
   public:
      LeastSquaresLine ();

      /**
       * Add a point to the regressed line and recalculate the parameters
       *
       * @param p             Point to be added to the line
       */
      inline void addPoint(Point p);

      /**
       * Remove a point from the regressed line and recalculate the parameters
       * This does not actually require the point was added before hand
       *
       * @param p             Point to be removeed to the line
       */
      inline void removePoint(Point p);

      /**
       * Get the line in ax+by+c=0 form
       * @param a             a
       * @param b             b
       * @param c             c
       */
      // TODO: Remove duplicate
      //bool getLineABC(long long *a, long long *b, long long *c);
      bool getLineABC(int64_t *a, int64_t *b, int64_t *c);

   private:
      int sumX2;
      int sumX;

      int sumY2;
      int sumY;

      int sumXY;

      int sum1; /* aka how many points */
};

#include "LeastSquaresLineInline.hpp"
