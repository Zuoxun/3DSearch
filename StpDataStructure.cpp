#include "StpDataStructure.h"

string ADVANCED_FACE::indexType(int index)
{
    if(face_outer_bounds.find(index) != face_outer_bounds.end()) return "FACE_OUTER_BOUND";
    else if(face_bounds.find(index) != face_bounds.end()) return "FACE_BOUND";
    else if(planes.find(index) != planes.end()) return "PLANE";
    else if(cylindrical_surfaces.find(index) != cylindrical_surfaces.end()) return "CYLINDRICAL_SURFACE";
    else if(conical_surfaces.find(index) != conical_surfaces.end()) return "CONICAL_SURFACE";
    else if(spherical_surfaces.find(index) != spherical_surfaces.end()) return "SPHERICAL_SURFACE";
    else if(toroidal_surfaces.find(index) != toroidal_surfaces.end()) return "TOROIDAL_SURFACE";
    return "";
}
//查询面类型
string ADVANCED_FACE::faceType()
{
    if(planes.find(face) != planes.end()) return "PLANE";
    else if(cylindrical_surfaces.find(face) != cylindrical_surfaces.end()) return "CYLINDRICAL_SURFACE";
    else if(conical_surfaces.find(face) != conical_surfaces.end()) return "CONICAL_SURFACE";
    else if(spherical_surfaces.find(face) != spherical_surfaces.end()) return "SPHERICAL_SURFACE";
    else if(toroidal_surfaces.find(face) != toroidal_surfaces.end()) return "TOROIDAL_SURFACE";
    return "";
}
//获取Z轴向量
Vector ADVANCED_FACE::getDirectionZ()
{
    string faceType = indexType(face);
    if(faceType == "PLANE") {
        auto pl_it = planes.find(face);
        auto ax_it = axis2_pacement_3ds.find(pl_it->second.axis2);
        auto dZ_it = directions.find(ax_it->second.directionZ);
        return dZ_it->second.getVector();
    }
    else if(faceType == "CYLINDRICAL_SURFACE") {
        auto cy_it = cylindrical_surfaces.find(face);
        auto ax_it = axis2_pacement_3ds.find(cy_it->second.axis2);
        auto dZ_it = directions.find(ax_it->second.directionZ);
        return dZ_it->second.getVector();
    }
    else if(faceType == "CONICAL_SURFACE") {
        auto co_it = conical_surfaces.find(face);
        auto ax_it = axis2_pacement_3ds.find(co_it->second.axis2);
        auto dZ_it = directions.find(ax_it->second.directionZ);
        return dZ_it->second.getVector();
    }
    else if(faceType == "SPHERICAL_SURFACE") {
        auto sp_it = spherical_surfaces.find(face);
        auto ax_it = axis2_pacement_3ds.find(sp_it->second.axis2);
        auto dZ_it = directions.find(ax_it->second.directionZ);
        return dZ_it->second.getVector();
    }
    else if(faceType == "TOROIDAL_SURFACE") {
        auto to_it = toroidal_surfaces.find(face);
        auto ax_it = axis2_pacement_3ds.find(to_it->second.axis2);
        auto dZ_it = directions.find(ax_it->second.directionZ);
        return dZ_it->second.getVector();
    }
    else {
        Vector vec;
        vec.x = 0;
        vec.y = 0;
        vec.z = 0;
        return vec;
    }
}

string FACE_OUTER_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}
//边界是否为单圆形
bool FACE_OUTER_BOUND::isCIRCLE()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        auto oe_it = oriented_edges.find(lp_it->second.edges[0]);
        auto ec_it = edge_curves.find(oe_it->second.curve);
        if(ec_it->second.indexType(ec_it->second.edge) == "CIRCLE") return true;
    }
    return false;
}
//边界是否为单圆形，返回CIRCLE索引号
bool FACE_OUTER_BOUND::isCIRCLE(int &circle_index)
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        auto oe_it = oriented_edges.find(lp_it->second.edges[0]);
        auto ec_it = edge_curves.find(oe_it->second.curve);
        if(ec_it->second.indexType(ec_it->second.edge) == "CIRCLE") {
            circle_index = ec_it->second.edge;
            return true;
        }
    }
    return false;
}
//边界是否为单边界（只包含一条边曲线）
bool FACE_OUTER_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}
//边界是否为顶点环
bool FACE_OUTER_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}
//边界是否为顶点环，返回VERTEX索引号
bool FACE_OUTER_BOUND::isVERTEX(int &vertex_index)
{
    auto lp_it = vertex_loops.find(loop);
    if(lp_it == vertex_loops.end()) return false;
    else {
        vertex_index = lp_it->second.vertex;
        return true;
    }
    return false;
}

string FACE_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}
//边界是否为单圆形
bool FACE_BOUND::isCIRCLE()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        auto oe_it = oriented_edges.find(lp_it->second.edges[0]);
        auto ec_it = edge_curves.find(oe_it->second.curve);
        if(ec_it->second.indexType(ec_it->second.edge) == "CIRCLE") return true;
    }
    return false;
}
//边界是否为单圆形，返回CIRCLE索引号
bool FACE_BOUND::isCIRCLE(int &circle_index)
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        auto oe_it = oriented_edges.find(lp_it->second.edges[0]);
        auto ec_it = edge_curves.find(oe_it->second.curve);
        if(ec_it->second.indexType(ec_it->second.edge) == "CIRCLE") {
            circle_index = ec_it->second.edge;
            return true;
        }
    }
    return false;
}
//边界是否为单边界（只包含一条边曲线）
bool FACE_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}
//边界是否为顶点环
bool FACE_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}
//边界是否为顶点环，返回VERTEX索引号
bool FACE_BOUND::isVERTEX(int &vertex_index)
{
    auto lp_it = vertex_loops.find(loop);
    if(lp_it == vertex_loops.end()) return false;
    else {
        vertex_index = lp_it->second.vertex;
        return true;
    }
    return false;
}

string AXIS2_PLACEMENT_3D::indexType(int index)
{
    if(circles.find(index) != circles.end()) return "CIRCLE";
    else if(planes.find(index) != planes.end()) return "PLANE";
    else if(cylindrical_surfaces.find(index) != cylindrical_surfaces.end()) return "CYLINDRICAL_SURFACE";
    else if(conical_surfaces.find(index) != conical_surfaces.end()) return "CONICAL_SURFACE";
    else if(spherical_surfaces.find(index) != spherical_surfaces.end()) return "SPHERICAL_SURFACE";
    else if(toroidal_surfaces.find(index) != toroidal_surfaces.end()) return "TOROIDAL_SURFACE";
    return "";
}

string VERTEX_LOOP::indexType(int index)
{
    if(face_outer_bounds.find(index) != face_outer_bounds.end()) return "FACE_OUTER_BOUND";
    else if(face_bounds.find(index) != face_bounds.end()) return "FACE_BOUND";
    return "";
}

string EDGE_LOOP::indexType(int index)
{
    if(face_outer_bounds.find(index) != face_outer_bounds.end()) return "FACE_OUTER_BOUND";
    else if(face_bounds.find(index) != face_bounds.end()) return "FACE_BOUND";
    return "";
}
//向上找到所在高级面的索引号
int ORIENTED_EDGE::findFace()
{
    auto lp_it = edge_loops.find(upIndex);
    if(lp_it->second.indexType(lp_it->second.upIndex) == "FACE_OUTER_BOUND") {
        auto fob_it = face_outer_bounds.find(lp_it->second.upIndex);
        return fob_it->second.upIndex;
    }
    else if(lp_it->second.indexType(lp_it->second.upIndex) == "FACE_BOUND") {
        auto fb_it = face_bounds.find(lp_it->second.upIndex);
        return fb_it->second.upIndex;
    }
}

string EDGE_CURVE::indexType(int index)
{
    if(lines.find(index) != lines.end()) return "LINE";
    else if(circles.find(index) != circles.end()) return "CIRCLE";
    return "";
}

string VERTEX_POINT::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_curves.find(index) != edge_curves.end()) return "EDGE_CURVE";
    return "";
}

string CARTESIAN_POINT::indexType(int index)
{
    if(vertex_points.find(index) != vertex_points.end()) return "VERTEX_POINT";
    else if(axis2_pacement_3ds.find(index) != axis2_pacement_3ds.end()) return "AXIS2_PLACEMENT_3D";
    else if(lines.find(index) != lines.end()) return "LINE";
    else if(circles.find(index) != circles.end()) return "CIRCLE";
    return "";
}
//转为Point点
Point CARTESIAN_POINT::toPoint()
{
    Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}

string DIRECTION::indexType(int index)
{
    if(vectors.find(index) != vectors.end()) return "VECTOR";
    else if(axis2_pacement_3ds.find(index) != axis2_pacement_3ds.end()) return "AXIS2_PLACEMENT_3D";
    return "";
}
//获取为三维向量
Vector DIRECTION::getVector()
{
    Vector vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}



//获得一个点到一条射线的垂足
Point GetFootOfPerpendicular(Point pt, Point begin, Vector vec)
{
	Point foot;
 
	double dx = vec.x;
	double dy = vec.y;
    double dz = vec.z;
	if(abs(dx) < 0.00000001 && abs(dy) < 0.00000001 && abs(dz) < 0.00000001 )
	{
		foot = begin;
		return foot;
	}
 
	double k = (pt.x - begin.x) * dx + (pt.y - begin.y) * dy + (pt.z - begin.z) * dz;
	k = k / ((dx*dx)+(dy*dy)+(dz*dz));
 
	foot.x = begin.x + k*dx;
	foot.y = begin.y + k*dy;
    foot.z = begin.z + k*dz;

	return foot;
}
//获得一个点到一条射线的垂线单位向量（垂线方向为从点指向射线）
Vector GetPerpendicular(Point pt, Point begin, Vector vec)
{
    Vector Perpendicular;

    Point foot = GetFootOfPerpendicular(pt, begin, vec);
    Perpendicular.x = foot.x - pt.x;
    Perpendicular.y = foot.y - pt.y;
    Perpendicular.z = foot.z - pt.z;

    return Perpendicular.toUnitVector();
}
//获得点a到点b的向量
Vector vec_ab(const Point &a, const Point &b)
{
    Vector vec;
    vec.x = b.x - a.x;
    vec.y = b.y - a.y;
    vec.z = b.z - a.z;
    return vec;
}
//获得两向量的夹角
double getAngle(Vector &a, Vector &b)
{
    double ab = a * b; //向量点乘内积
    double _a_ = sqrt(a.x * a.x + a.y * a.y + a.z * a.z); //向量a的模
    double _b_ = sqrt(b.x * b.x + b.y * b.y + b.z * b.z); //向量b的模

    double Angle = acos(ab / (_a_ * _b_));
    return Angle; //返回夹角
}
//获得两向量的夹角（的绝对值）
double getAbsAngle(Vector &a, Vector &b)
{
    double ab = a * b; //向量点乘内积
    double _a_ = sqrt(a.x * a.x + a.y * a.y + a.z * a.z); //向量a的模
    double _b_ = sqrt(b.x * b.x + b.y * b.y + b.z * b.z); //向量b的模

    double Angle = acos(ab / (_a_ * _b_));
    return fabs(Angle); //返回夹角的绝对值
}
//转换为单位向量
Vector Vector::toUnitVector()
{
    double len = sqrt(x*x + y*y + z*z);
    if(len == 0) {
        Vector vec = *this;
        return vec;
    }
    x = x / len;
    y = y / len;
    z = z / len;
    Vector vec = *this;
    return vec;
}
//转换为单位向量
Vector toUnitVector(const Vector &v)
{
    double len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if(len == 0) {
        Vector vec(v.x, v.y, v.z);
        return vec;
    }
    Vector vec;
    vec.x = v.x / len;
    vec.y = v.y / len;
    vec.z = v.z / len;
    return vec;
}
//转换为相反向量
Vector Vector::toOppositeVector()
{
    x = -x;
    y = -y;
    z = -z;
    Vector vec = *this;
    return vec;
}
//转换为相反向量
Vector toOppositeVector(const Vector &v)
{
    Vector vec;
    vec.x = -v.x;
    vec.y = -v.y;
    vec.z = -v.z;
    return vec;
}