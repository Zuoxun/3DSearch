#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <math.h>
using namespace std;

#define PI acos(-1)  //π
#define V0 Vector(0,0,0)  //零向量
#define AnglePrecision 0.001  //角度计算的精度
#define Precision 0.001  //长度、坐标等计算的精度

//模糊判断两个角度是否相等（精度在0.001内就认为相等）
bool anglesAreEqual(double angle1, double angle2);
//模糊判断两个double数值（长度、坐标值）是否相等（精度在0.001内就认为相等）
bool isEqual(double num1, double num2);

struct Point
{
public:
    double x,y,z;
    Point(){};
    Point (double x, double y, double z): x(x),y(y),z(z) {};
};
struct Vector
{
public:
    double x,y,z;
    Vector(){};
    Vector (double x, double y, double z): x(x),y(y),z(z) {};
    Vector toUnitVector(); //转换为单位向量
    Vector toOppositeVector(); //转换为相反向量
    //向量相加
    Vector operator+(const Vector &b)
    {
        Vector ret;
        ret.x = this->x + b.x;
        ret.y = this->y + b.y;
        ret.z = this->z + b.z;
        return ret;
    }
    //向量相减
    Vector operator-(const Vector &b)
    {
        Vector ret;
        ret.x = this->x - b.x;
        ret.y = this->y - b.y;
        ret.z = this->z - b.z;
        return ret;
    }
    //向量点乘
    double operator*(const Vector &b)
    {
        double ret;
        double x = this->x * b.x;
        double y = this->y * b.y;
        double z = this->z * b.z;
        ret = x + y + z;
        return ret;
    }
    //向量叉乘（遵守右手定则）（与点乘不同）(叉乘的顺序不同，得到的结果向量相反)
    Vector operator^(const Vector &b)
    {
        Vector ret;
        ret.x = this->y * b.z - b.y * this->z;
        ret.y = b.x * this->z - this->x * b.z;
        ret.z = this->x * b.y - b.x * this->y;
        return ret;
    }
    //向量 * double
    Vector operator*(double b)
    {
        Vector ret;
        ret.x = this->x * b;
        ret.y = this->y * b;
        ret.z = this->z * b;
        return ret;
    }
    //向量 / double
    Vector operator/(double b)
    {
        Vector ret;
        ret.x = this->x / b;
        ret.y = this->y / b;
        ret.z = this->z / b;
        return ret;
    }
    //向量相等（精度在0.001内就认为相等）
    bool operator==(const Vector &b)
    {
        if(!isEqual(this->x, b.x)) return false;
        if(!isEqual(this->y, b.y)) return false;
        if(!isEqual(this->z, b.z)) return false;
        return true;
    }
};
//获得一个点到一条射线的垂足
Point GetFootOfPerpendicular(
    Point pt, //射线外一点
    Point begin, //射线起点
    Vector vec); //射线向量
//获得一个点到一条射线的垂线单位向量（垂线方向为从点指向射线）
Vector GetPerpendicular(
    Point pt, //射线外一点
    Point begin, //射线起点
    Vector vec); //射线向量
//获得点A到点B的向量
Vector vec_AB(const Point &A, const Point &B);
//获得点A到点B的距离
double distance_AB(const Point &A, const Point &B);
//获得两向量的夹角
double getAngle(const Vector &a, const Vector &b);
//获得两向量的夹角（的绝对值）
double getAbsAngle(const Vector &a, const Vector &b);
//转换为单位向量
Vector toUnitVector(const Vector &v);
//转换为相反向量
Vector toOppositeVector(const Vector &v);
//判断两向量是否平行
bool isParallel(Vector a, Vector b);
//判断两向量是否垂直
bool isVertical(Vector a, Vector b);


//公共边
struct CommonEdge
{
public:
    int index; //公共边曲线索引号
    string edgeType; //边曲线类型（"LINE","CIRCLE"）
    int concavity; //凹凸性（凹凸性1为凸，0为凹,-1为非凹非凸）
    double angle; //夹角

    CommonEdge (int index, string edgeType, int concavity, double angle):index(index),edgeType(edgeType),concavity(concavity),angle(angle) {};
    Vector getVector(); //获得直线边的向量（圆曲线的话返回零向量）
};
//判断两公共直线边是否平行
bool isParallel(const CommonEdge &a, const CommonEdge &b);
//判断两公共直线边是否垂直
bool isVertical(const CommonEdge &a, const CommonEdge &b);


/***
每种实体都用不同的类，每类都有一个map容器
建立索引
***/
class CLOSED_SHELL
{
public:
    int index;
    string text;
    vector<int> faces;

    CLOSED_SHELL(){}
};

class ADVANCED_FACE
{
public:
    int index;
    int upIndex;
    string text;
    vector<int> bounds;
    int face;
    bool flag;

    map<int, CommonEdge> adjacentFaces; //map<相邻面索引号, 公共边曲线>
    int isUsed = 0; //特征识别标志位（0为未被识别为特征，1为被识别为特征，2为被识别为特征关键面）
    int isVisited = 0; //属性邻接图搜索标志位（0为未访问过，1为访问过）

    string faceType; //面类型
    ADVANCED_FACE(){}
    string indexType(int index);
    //string faceType(); //查询面类型
    bool setUse(int n); //设置特征识别标志位（0为未被识别为特征，1为被识别为特征，2为被识别为特征关键面）
    Vector getDirectionZ(); //获取Z轴向量
    int concaveCommonEdge_number(); //获取凹公共边个数
    int concaveCommonLineEdge_number(); //获取凹公共直线边个数
    vector<map<int, CommonEdge>::iterator> get_concaveAdjacentFaces(); //获取所有凹相邻面的迭代器
    bool get_4concaveAdjacentFaces_closedLoopOutside(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces); //尝试找到一个尺寸最大的由4个凹公共直线边组成的闭环边界（比较边长），返回这四个凹相邻面的迭代器
    bool get_6concaveAdjacentFaces_closedLoopOutside(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces); //尝试找到一个尺寸最大的由6个同样长度的凹公共直线边组成的闭环边界（比较边长），返回这六个凹相邻面的迭代器
    int commonEdge_number_withFace(int faceIndex); //获取与一个高级面的公共边数量
    CommonEdge* getCommonEdge_withAdjacentFace(int faceIndex); //获取与一个相邻面的公共边指针
};
 
class FACE_OUTER_BOUND
{
public:
    int index;
    int upIndex;
    string text;
    int loop;
    bool flag;

    FACE_OUTER_BOUND(){}
    string indexType(int index);
    bool isCIRCLE(); //边界是否为单圆形
    bool isCIRCLE(int &circle_index); //边界是否为单圆形，返回CIRCLE索引号
    bool isSingleEdge(); //边界是否为单边界（只包含一条边曲线）
    bool isVERTEX(); //边界是否为顶点环
    bool isVERTEX(int &vertex_index); //边界是否为顶点环，返回VERTEX索引号
    vector<int> getAllEdges(); //获取边界包含的所有边曲线的索引号
};

class FACE_BOUND
{
public:
    int index;
    int upIndex;
    string text;
    int loop;
    bool flag;

    FACE_BOUND(){}
    string indexType(int index);
    bool isCIRCLE(); //边界是否为单圆形
    bool isCIRCLE(int &circle_index); //边界是否为单圆形，返回CIRCLE索引号
    bool isSingleEdge(); //边界是否为单边界（只包含一条边曲线）
    bool isVERTEX(); //边界是否为顶点环
    bool isVERTEX(int &vertex_index); //边界是否为顶点环，返回VERTEX索引号
    vector<int> getAllEdges(); //获取边界包含的所有边曲线的索引号
};

class PLANE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;

    PLANE(){}
};

class CYLINDRICAL_SURFACE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;
    double radius;

    CYLINDRICAL_SURFACE(){}
};

class CONICAL_SURFACE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;
    double radius;
    double coneAngle;

    CONICAL_SURFACE(){}
};

class SPHERICAL_SURFACE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;
    double radius;

    SPHERICAL_SURFACE(){}
};

class TOROIDAL_SURFACE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;
    double major_radius;
    double minor_radius;

    TOROIDAL_SURFACE(){}
};

class AXIS2_PLACEMENT_3D
{
public:
    int index;
    int upIndex;
    string text;
    int point;
    int directionZ, directionX;

    AXIS2_PLACEMENT_3D(){}
    string indexType(int index);
};

class VERTEX_LOOP
{
public:
    int index;
    int upIndex;
    string text;
    int vertex;

    VERTEX_LOOP(){}
    string indexType(int index);
};

class EDGE_LOOP
{
public:
    int index;
    int upIndex;
    string text;
    vector<int> edges;

    EDGE_LOOP(){}
    string indexType(int index);
};

class ORIENTED_EDGE
{
public:
    int index;
    int upIndex;
    string text;
    int curve;
    bool flag;

    ORIENTED_EDGE(){}
    int findFace(); //向上找到所在高级面的索引号
};

class EDGE_CURVE
{
public:
    int index;
    vector<int> upIndexes; //寻找相邻边的关键
    string text;
    int vertex1;
    int vertex2;
    int edge;
    bool flag;

    string edgeType; //边曲线类型（"LINE","CIRCLE"）
    EDGE_CURVE(){}
    string indexType(int index);
    double length(); //获取直线边曲线的长度（若不是直线，返回0）
};

class VERTEX_POINT
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int point;

    VERTEX_POINT(){}
    string indexType(int index);
};

class CARTESIAN_POINT
{
public:
    int index;
    int upIndex;
    string text;
    double x, y, z;

    CARTESIAN_POINT(){}
    string indexType(int index);
    Point toPoint(); //转为Point点
};

class LINE
{
public:
    int index;
    int upIndex;
    string text;
    int point;
    int vector;

    LINE(){}
};

class VECTOR
{
public:
    int index;
    int upIndex;
    string text;
    int direction;
    double n;  //invalid

    VECTOR(){}
};

class DIRECTION
{
public:
    int index;
    int upIndex;
    string text;
    double x, y, z;

    DIRECTION(){}
    string indexType(int index);
    Vector getVector(); //获取为三维向量
};

class CIRCLE
{
public:
    int index;
    int upIndex;
    string text;
    int axis2;
    double radius;

    CIRCLE(){}
};

//数据结构组织容器
map<int, CLOSED_SHELL> closed_shells;
map<int, ADVANCED_FACE> advanced_faces;
map<int, FACE_OUTER_BOUND> face_outer_bounds;
map<int, FACE_BOUND> face_bounds;
map<int, PLANE> planes;
map<int, CYLINDRICAL_SURFACE> cylindrical_surfaces;
map<int, CONICAL_SURFACE> conical_surfaces;
map<int, SPHERICAL_SURFACE> spherical_surfaces;
map<int, TOROIDAL_SURFACE> toroidal_surfaces;
map<int, AXIS2_PLACEMENT_3D> axis2_pacement_3ds;
map<int, VERTEX_LOOP> vertex_loops;
map<int, EDGE_LOOP> edge_loops;
map<int, ORIENTED_EDGE> oriented_edges;
map<int, EDGE_CURVE> edge_curves;
map<int, VERTEX_POINT> vertex_points;
map<int, CARTESIAN_POINT> cartesian_points;
map<int, LINE> lines;
map<int, VECTOR> vectors;
map<int, DIRECTION> directions;
map<int, CIRCLE> circles;

//关键词库查询
set<string> keywords{
    "CLOSED_SHELL",
    "ADVANCED_FACE",
    "FACE_OUTER_BOUND",
    "FACE_BOUND",
    "PLANE",
    "CYLINDRICAL_SURFACE",
    "CONICAL_SURFACE",
    "SPHERICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "AXIS2_PLACEMENT_3D",
    "VERTEX_LOOP",
    "EDGE_LOOP",
    "ORIENTED_EDGE",
    "EDGE_CURVE",
    "VERTEX_POINT",
    "CARTESIAN_POINT",
    "LINE",
    "VECTOR",
    "DIRECTION",
    "CIRCLE"
};
