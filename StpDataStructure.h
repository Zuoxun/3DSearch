#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
using namespace std;
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

    vector<int> adjacentFaces;

    ADVANCED_FACE(){}
    string indexType(int index);
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
    bool isCIRCLE();
    bool isCIRCLE(int &circle_index);
    bool isSingleEdge();
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
    bool isCIRCLE();
    bool isCIRCLE(int &circle_index);
    bool isSingleEdge();
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
};

class EDGE_CURVE
{
public:
    int index;
    vector<int> upIndexes; //寻找相邻边的关键点
    string text;
    int vertex1;
    int vertex2;
    int edge;
    bool flag;

    EDGE_CURVE(){}
    string indexType(int index);
};

class VERTEX_POINT
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int point;

    VERTEX_POINT(){}
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
