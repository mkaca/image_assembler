/*********************************************************************
Copyright (c) 2022, Cobalt Robotics, Inc.
Author: Michael Kaca
 *********************************************************************/

#ifndef IMAGE_ASSEMBLER_INTENSITY_PIXEL_
#define IMAGE_ASSEMBLER_INTENSITY_PIXEL_

namespace image_assembler
{

struct PointDataXYDI
{
  double x;
  double y;
  double depth;
  unsigned int intensity;
};

}  // namespace image_assembler

#endif  // IMAGE_ASSEMBLER_INTENSITY_PIXEL_
