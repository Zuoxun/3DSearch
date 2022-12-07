#pragma once

#include <math.h>
#include "StpDataStructure.h"

//计算两点之间距离
double pointDistance(int p1, int p2);
//计算两点之间距离
double pointDistance(double x1, double y1, double z1, double x2, double y2, double z2);

//两面相邻情况判断（入参：两个高级面一个边曲线的索引号）
int adjacencySituation(int &face1, int &face2, int edge);

//建立属性邻接图AAG
void makeAAG();

//提取圆柱和圆孔
void cylinder_and_circularHole();

//提取圆锥、圆锥孔、圆台、圆台孔
void cone_and_conicalHole();