#include "tabs/tab.hpp"

#include "perception/vision/other/YUV.hpp"
#include "utils/classifier.hpp"


QRgb Tab::getRGB(unsigned int col,
                 unsigned int row,
                 const uint8_t *yuv,
                 int num_cols) {

   int y = gety(yuv, row, col, num_cols);
   int u = getu(yuv, row, col, num_cols);
   int v = getv(yuv, row, col, num_cols);


   return Classifier::yuv2rgb(gety(yuv, row, col, num_cols),
                              getu(yuv, row, col, num_cols),
                              getv(yuv, row, col, num_cols));
}

void Tab::tabSelected()
{
}

void Tab::tabDeselected()
{
}
