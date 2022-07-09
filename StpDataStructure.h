#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <map>
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

    ADVANCED_FACE(){}
};

class FACE_OUTER_BOUND
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int loop;
    bool flag;

    FACE_OUTER_BOUND(){}
};

class FACE_BOUND
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int loop;
    bool flag;

    FACE_BOUND(){}
};

class PLANE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int axis2;

    PLANE(){}
};

class CYLINDRICAL_SURFACE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int axis2;
    double radius;

    CYLINDRICAL_SURFACE(){}
};

class CONICAL_SURFACE
{
public:
    int index;
    vector<int> upIndexes;
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
    vector<int> upIndexes;
    string text;
    int axis2;
    double radius;

    SPHERICAL_SURFACE(){}
};

class TOROIDAL_SURFACE
{
public:
    int index;
    vector<int> upIndexes;
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
    vector<int> upIndexes;
    string text;
    int point;
    int directionZ, directionX;

    AXIS2_PLACEMENT_3D(){}
};

class EDGE_LOOP
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    vector<int> edges;

    EDGE_LOOP(){}
};

class ORIENTED_EDGE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int curve;
    bool flag;

    ORIENTED_EDGE(){}
};

class EDGE_CURVE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int vertex1;
    int vertex2;
    int edge;
    bool flag;

    EDGE_CURVE(){}
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
    vector<int> upIndexes;
    string text;
    double x, y, z;

    CARTESIAN_POINT(){}
};

class LINE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int point;
    int vector;

    LINE(){}
};

class VECTOR
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int direction;
    double n;

    VECTOR(){}
};

class DIRECTION
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    double x, y, z;

    DIRECTION(){}
};

class CIRCLE
{
public:
    int index;
    vector<int> upIndexes;
    string text;
    int axis2;
    double radius;

    CIRCLE(){}
};

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

string findIndexType(int index)
{
    if(closed_shells.count(index) > 0) {
        return "closed_shells";
    }
    if(advanced_faces.count(index) > 0) {
        return "advanced_faces";
    }
    if(face_outer_bounds.count(index) > 0) {
        return "face_outer_bounds";
    }
    if(face_bounds.count(index) > 0) {
        return "face_bounds";
    }
    if(planes.count(index) > 0) {
        return "planes";
    }
    if(cylindrical_surfaces.count(index) > 0) {
        return "cylindrical_surfaces";
    }
    if(conical_surfaces.count(index) > 0) {
        return "conical_surfaces";
    }
    if(spherical_surfaces.count(index) > 0) {
        return "spherical_surfaces";
    }
    if(toroidal_surfaces.count(index) > 0) {
        return "toroidal_surfaces";
    }
    if(axis2_pacement_3ds.count(index) > 0) {
        return "axis2_pacement_3ds";
    }
    if(edge_loops.count(index) > 0) {
        return "edge_loops";
    }
    if(oriented_edges.count(index) > 0) {
        return "oriented_edges";
    }
    if(edge_curves.count(index) > 0) {
        return "edge_curves";
    }
    if(vertex_points.count(index) > 0) {
        return "vertex_points";
    }
    if(cartesian_points.count(index) > 0) {
        return "cartesian_points";
    }
    if(lines.count(index) > 0) {
        return "lines";
    }
    if(vectors.count(index) > 0) {
        return "vectors";
    }
    if(directions.count(index) > 0) {
        return "directions";
    }
}