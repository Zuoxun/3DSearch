#pragma once

#include <math.h>
#include "StpDataStructure.h"

//计算两点之间距离
double pointDistance(map<int, CARTESIAN_POINT>::iterator it1, map<int, CARTESIAN_POINT>::iterator it2);
double pointDistance(double x1, double y1, double z1, double x2, double y2, double z2);

//提取圆柱和圆孔
void cylinder_and_circularHole();