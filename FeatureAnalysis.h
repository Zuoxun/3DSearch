#pragma once

#include "StpDataStructure.h"

//计算两点之间距离
double pointDistance(int p1, int p2);
//计算两点之间距离
double pointDistance(double x1, double y1, double z1, double x2, double y2, double z2);

//两面相邻情况判断（入参：两个高级面一个边曲线的索引号）（如果是一个柱面+一个平面，两个面索引调整为face1平面，face2柱面）
int adjacencySituation(int &face1, int &face2, int edge);

//建立属性邻接图AAG
void makeAAG();

//提取圆柱和圆孔
void cylinder_and_circularHole();

//提取圆环槽
void annularSlot();

//提取圆锥、圆锥孔、圆台、圆台孔
void cone_and_conicalHole();

//设置全部特征识别标志位（0为未被识别为特征，1为被识别为特征，2为被识别为特征关键面）
bool setUse(const int &n);

//设置全部属性邻接图搜索标志位（0为未访问过，1为访问过）
bool setVisit(const int &n);

//提取型腔
void closedPocket();
/*侧相邻面重排序+获得互相间的公共直线边指针

说明：1.验证平面的四个凹相邻面是否符合型腔的四个凹相邻面两两之间有且仅有一个公共直线边的相邻特征。
    2.调整存储的四个凹相邻面的顺序（顺序为：第0个、第1个为一对不相邻的对位面，第2个、第3个为另一对不相邻的对位面）。
    3.返回四个公共直线边指针（顺序为：第0个、第2个相邻面之间的公共直线边，第0个、第3个相邻面之间…，第1个、第2个相邻面之间…，第1个、第3个相邻面之间…）
*/
bool reorderAdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges);
//验证一组向量是否互相之间都平行
bool allParallel(const vector<Vector> &vecs);
//验证一组公共边是否属于并组成一个边环
bool formAnEdgeLoop(const vector<map<int, CommonEdge>::iterator> &outsideAdjacentFaces);

//六角型腔
void hexagonalClosedPocket();
/*侧相邻面重排序+获得互相间的公共直线边指针

说明：1.验证平面的六个凹相邻面是否符合六角型腔的六个凹相邻面两两之间有且仅有一个公共直线边的相邻特征。
    2.调整存储的六个凹相邻面的顺序（顺序为：以顺时针或逆时针顺序，第0、1、2、3、4、5个面依次相邻）。
    3.返回六个公共直线边指针（顺序为：第0、1、2、3、4、5个面依次每两个相邻面之间的公共直线边）
*/
bool reorder6AdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges);

//提取方孔
void squareHole();

//提取六角孔
void hexagonalHole();

//提取盲槽
void openPocket();

//提取通槽
void throughSlot();

//六棱柱（螺母）
void hexagonalPrism();

//提取封闭键槽
void keySlots();

/*获得2个直线边侧相邻面与2个圆边侧相邻面互相间的公共直线边指针

说明：1.验证平面的两个凹直线边相邻面与两个凹圆边相邻面是否符合：键槽的四个凹相邻面两两之间有且仅有一个公共直线边，且柱面与平面交替相邻的相邻特征。
    2.存储的四个凹相邻面的顺序为：第0个、第1个为一对不相邻的对位平面，第2个、第3个为一对不相邻的对位柱面。
    3.返回四个公共直线边指针（顺序为：第0个、第2个相邻面之间的公共直线边，第0个、第3个相邻面之间…，第1个、第2个相邻面之间…，第1个、第3个相邻面之间…）
*/
bool keySlot_getSideCommonEdges(const vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, const vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges);

//提取开放键槽
void openKeySlot();

//提取U形通槽
void UshapedSlot();