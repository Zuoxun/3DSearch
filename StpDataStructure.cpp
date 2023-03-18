#include "StpDataStructure.h"


bool anglesAreEqual(double angle1, double angle2)
{
    if(max(angle1, angle2) - min(angle1, angle2) < AnglePrecision) return true;
    else return false;
}
bool isEqual(double num1, double num2)
{
    if(max(num1, num2) - min(num1, num2) < Precision) return true;
    else return false;
}


//���һ���㵽һ�����ߵĴ���
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
//���һ���㵽һ�����ߵĴ��ߵ�λ���������߷���Ϊ�ӵ�ָ�����ߣ�
Vector GetPerpendicular(Point pt, Point begin, Vector vec)
{
    Vector Perpendicular;

    Point foot = GetFootOfPerpendicular(pt, begin, vec);
    Perpendicular.x = foot.x - pt.x;
    Perpendicular.y = foot.y - pt.y;
    Perpendicular.z = foot.z - pt.z;

    return Perpendicular.toUnitVector();
}
//��õ�A����B������
Vector vec_AB(const Point &A, const Point &B)
{
    Vector vec;
    vec.x = B.x - A.x;
    vec.y = B.y - A.y;
    vec.z = B.z - A.z;
    return vec;
}
//��õ�A����B�ľ���
double distance_AB(const Point &A, const Point &B)
{
    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2));
}
//����������ļн�
double getAngle(Vector &a, Vector &b)
{
    double ab = a * b; //��������ڻ�
    double _a_ = sqrt(a.x * a.x + a.y * a.y + a.z * a.z); //����a��ģ
    double _b_ = sqrt(b.x * b.x + b.y * b.y + b.z * b.z); //����b��ģ

    double Angle = acos(ab / (_a_ * _b_));
    return Angle; //���ؼн�
}
//����������ļнǣ��ľ���ֵ��
double getAbsAngle(Vector &a, Vector &b)
{
    double ab = a * b; //��������ڻ�
    double _a_ = sqrt(a.x * a.x + a.y * a.y + a.z * a.z); //����a��ģ
    double _b_ = sqrt(b.x * b.x + b.y * b.y + b.z * b.z); //����b��ģ

    double Angle = acos(ab / (_a_ * _b_));
    return fabs(Angle); //���ؼнǵľ���ֵ
}
//ת��Ϊ��λ����
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
//ת��Ϊ��λ����
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
//ת��Ϊ�෴����
Vector Vector::toOppositeVector()
{
    x = -x;
    y = -y;
    z = -z;
    Vector vec = *this;
    return vec;
}
//ת��Ϊ�෴����
Vector toOppositeVector(const Vector &v)
{
    Vector vec;
    vec.x = -v.x;
    vec.y = -v.y;
    vec.z = -v.z;
    return vec;
}
//�ж��������Ƿ�ƽ��
bool isParallel(Vector a, Vector b)
{
    if((a ^ b) == V0) return true;
    else return false;
}
//�ж��������Ƿ�ֱ
bool isVertical(Vector a, Vector b)
{
    if(isEqual((a * b), 0)) return true;
    else return false;
}
//���ֱ�߱ߵ�������Բ���ߵĻ�������������
Vector CommonEdge::getVector()
{
    auto ec_it = edge_curves.find(this->index);
    auto ln_it = lines.find(ec_it->second.edge);
    if(ln_it == lines.end()) return V0; //�����߲���ֱ�ߣ�����������
    auto vec_it = vectors.find(ln_it->second.vector);
    auto dir_it = directions.find(vec_it->second.direction);
    return dir_it->second.getVector();
}
//�ж�������ֱ�߱��Ƿ�ƽ��
bool isParallel(CommonEdge &a, CommonEdge &b)
{
    Vector v1 = a.getVector();
    Vector v2 = b.getVector();
    if(v1 == V0 || v2 == V0) return false; //�κ�һ�������߲���ֱ�ߣ����ж�Ϊ��ƽ�й�ϵ
    if((v1 ^ v2) == V0) return true;
    else return false;
}
//�ж�������ֱ�߱��Ƿ�ֱ
bool isVertical(CommonEdge &a, CommonEdge &b)
{
    Vector v1 = a.getVector();
    Vector v2 = b.getVector();
    if(v1 == V0 || v2 == V0) return false; //�κ�һ�������߲���ֱ�ߣ����ж�Ϊ��ƽ�й�ϵ
    if(isEqual((v1 * v2), 0)) return true;
    else return false;
}


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
// //��ѯ������
// string ADVANCED_FACE::faceType()
// {
//     if(planes.find(face) != planes.end()) return "PLANE";
//     else if(cylindrical_surfaces.find(face) != cylindrical_surfaces.end()) return "CYLINDRICAL_SURFACE";
//     else if(conical_surfaces.find(face) != conical_surfaces.end()) return "CONICAL_SURFACE";
//     else if(spherical_surfaces.find(face) != spherical_surfaces.end()) return "SPHERICAL_SURFACE";
//     else if(toroidal_surfaces.find(face) != toroidal_surfaces.end()) return "TOROIDAL_SURFACE";
//     return "";
// }
//��������ʶ���־λ��0Ϊδ��ʶ��Ϊ������1Ϊ��ʶ��Ϊ������2Ϊ��ʶ��Ϊ�����ؼ��棩
bool ADVANCED_FACE::setUse(int n)
{
    if(n != 0 && n != 1 && n != 2) return false;
    this->isUsed = n;
    return true;
}
//��ȡZ������
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
//��ȡ�������߸���
int ADVANCED_FACE::concaveCommonEdge_number()
{
    int num = 0;
    for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
    {
        if(it->second.concavity == 0) num++;
    }
    return num;
}
//��ȡ������ֱ�߱߸���
int ADVANCED_FACE::concaveCommonLineEdge_number()
{
    int num = 0;
    for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
    {
        if(it->second.concavity == 0 && it->second.edgeType == "LINE") num++;
    }
    return num;
}
//��ȡ���а�������ĵ�����
vector<map<int, CommonEdge>::iterator> ADVANCED_FACE::get_concaveAdjacentFaces()
{
    vector<map<int, CommonEdge>::iterator> concaveAdjacentFaces;
    for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
    {
        if(it->second.concavity == 0) {
            concaveAdjacentFaces.emplace_back(it);
        }
    }
    return concaveAdjacentFaces;
}
//�����ҵ�һ���ߴ�������4��������ֱ�߱���ɵıջ��߽磨�Ƚϱ߳������������ĸ���������ĵ�����
bool ADVANCED_FACE::get_4concaveAdjacentFaces_closedLoopOutside(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces)
{
    double maxlen = 0; //�ջ��߽�����ߵĳ���
    for(int i = 0; i < bounds.size(); i++)
    {
        vector<map<int, CommonEdge>::iterator> temp; //�ñպϱ߽�����İ�����ֱ�߱�/��������
        vector<int> Edges; //�߽���������б����ߵ�������
        auto fob_it = face_outer_bounds.find(bounds[i]);
        auto fb_it = face_bounds.find(bounds[i]);
        if(fob_it != face_outer_bounds.end()) Edges = fob_it->second.getAllEdges();
        else Edges = fb_it->second.getAllEdges();

        if(Edges.size() != 4) continue; //�ñջ��߽������4�����������

        //��4���߱����ǰ�����ֱ�߱�
        bool f = true;
        for(int j = 0; j < Edges.size(); j++)
        {
            bool ff = false;
            for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
            {
                if(Edges[j] == it->second.index && it->second.concavity == 0 && it->second.edgeType == "LINE") {
                    ff = true;
                    temp.emplace_back(it);
                    break;
                }
            }
            if(ff == false) {
                f = false;
                break;
            }
        }
        if(f == false) continue;

        //�Ƚ�ѡ��ߴ����ıջ��߽�
        double temp_maxlen = 0;
        for(int k = 0; k < temp.size(); k++)
        {
            auto ec_it = edge_curves.find(temp[k]->second.index);
            auto vp_it1 = vertex_points.find(ec_it->second.vertex1);
            auto vp_it2 = vertex_points.find(ec_it->second.vertex2);
            auto cp_it1 = cartesian_points.find(vp_it1->second.point);
            auto cp_it2 = cartesian_points.find(vp_it2->second.point);
            Point P1 = cp_it1->second.toPoint();
            Point P2 = cp_it2->second.toPoint();
            double len = distance_AB(P1, P2);
            temp_maxlen = max(temp_maxlen, len);
        }
        if(temp_maxlen > maxlen) {
            concaveAdjacentFaces = temp;
            maxlen = temp_maxlen;
        }
    }

    if(maxlen > 0) return true; //�ҵ��˷��������ıպϱ߽�
}
//��ȡ��һ���߼���Ĺ���������
int ADVANCED_FACE::commonEdge_number_withFace(int faceIndex)
{
    int n = 0;
    for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
    {
        if(it->first == faceIndex) n++;
    }
    return n;
}
//��ȡ��һ��������Ĺ�����ָ��
CommonEdge* ADVANCED_FACE::getCommonEdge_withAdjacentFace(int faceIndex)
{
    CommonEdge* commonEdge;
    for(auto it = adjacentFaces.begin(); it != adjacentFaces.end(); it++)
    {
        if(it->first == faceIndex) return &it->second;
    }
    return nullptr;
}

string FACE_OUTER_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}
//�߽��Ƿ�Ϊ��Բ��
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
//�߽��Ƿ�Ϊ��Բ�Σ�����CIRCLE������
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
//�߽��Ƿ�Ϊ���߽磨ֻ����һ�������ߣ�
bool FACE_OUTER_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}
//�߽��Ƿ�Ϊ���㻷
bool FACE_OUTER_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}
//�߽��Ƿ�Ϊ���㻷������VERTEX������
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
//��ȡ�߽���������б����ߵ�������
vector<int> FACE_OUTER_BOUND::getAllEdges()
{
    vector<int> Edges;
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return Edges;
    for(int i = 0; i < lp_it->second.edges.size(); i++)
    {
        auto oe_it = oriented_edges.find(lp_it->second.edges[i]);
        Edges.emplace_back(oe_it->second.curve);
    }
    return Edges;
}

string FACE_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}
//�߽��Ƿ�Ϊ��Բ��
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
//�߽��Ƿ�Ϊ��Բ�Σ�����CIRCLE������
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
//�߽��Ƿ�Ϊ���߽磨ֻ����һ�������ߣ�
bool FACE_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}
//�߽��Ƿ�Ϊ���㻷
bool FACE_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}
//�߽��Ƿ�Ϊ���㻷������VERTEX������
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
//��ȡ�߽���������б����ߵ�������
vector<int> FACE_BOUND::getAllEdges()
{
    vector<int> Edges;
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return Edges;
    for(int i = 0; i < lp_it->second.edges.size(); i++)
    {
        auto oe_it = oriented_edges.find(lp_it->second.edges[i]);
        Edges.emplace_back(oe_it->second.curve);
    }
    return Edges;
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
//�����ҵ����ڸ߼����������
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
//��ȡֱ�߱����ߵĳ��ȣ�������ֱ�ߣ�����0��
double EDGE_CURVE::length()
{
    if(this->edgeType != "LINE") return 0; //������ֱ�ߣ�����0
    auto LINE_VertexP1 = vertex_points.find(this->vertex1); //�����߶���1
    auto LINE_VertexP2 = vertex_points.find(this->vertex2); //�����߶���2
    auto LINE_CartesianP1 = cartesian_points.find(LINE_VertexP1->second.point); //�����ߵѿ�����1
    auto LINE_CartesianP2 = cartesian_points.find(LINE_VertexP2->second.point); //�����ߵѿ�����2
    Point P1 = LINE_CartesianP1->second.toPoint();
    Point P2 = LINE_CartesianP2->second.toPoint();
    double length = distance_AB(P1, P2); //ֱ�߱����߳���
    return length;
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
//תΪPoint��
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
//��ȡΪ��ά����
Vector DIRECTION::getVector()
{
    Vector vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}



