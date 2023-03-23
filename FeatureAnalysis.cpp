#include "FeatureAnalysis.h"

double pointDistance(int p1, int p2)
{
    auto it1 = cartesian_points.find(p1);
    auto it2 = cartesian_points.find(p2);
    double x1 = it1->second.x;
    double y1 = it1->second.y;
    double z1 = it1->second.z;
    double x2 = it2->second.x;
    double y2 = it2->second.y;
    double z2 = it2->second.z;
    double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
    return d;
}

double pointDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
    return d;
}

int adjacencySituation(int &face1, int &face2, int edge)
{
    auto ad_it1 = advanced_faces.find(face1);
    auto ad_it2 = advanced_faces.find(face2);
    string faceType1 = ad_it1->second.indexType(ad_it1->second.face);
    string faceType2 = ad_it2->second.indexType(ad_it2->second.face);
    auto ec_it = edge_curves.find(edge);
    string edgeType = ec_it->second.indexType(ec_it->second.edge);

    if(faceType1 == "PLANE" && faceType2 == "PLANE" && edgeType == "LINE") return 1;
    else if(faceType1 == "PLANE" && faceType2 == "CYLINDRICAL_SURFACE" && edgeType == "LINE") return 2;
    else if(faceType1 == "CYLINDRICAL_SURFACE" && faceType2 == "PLANE" && edgeType == "LINE") { //��face1������face2��ƽ�棬��Ҫ����������˳��
        swap(face1, face2);
        return 2;
    }
    else if(faceType1 == "PLANE" && faceType2 == "CYLINDRICAL_SURFACE" && edgeType == "CIRCLE") return 3;
    else if(faceType1 == "CYLINDRICAL_SURFACE" && faceType2 == "PLANE" && edgeType == "CIRCLE") { //��face1������face2��ƽ�棬��Ҫ����������˳��
        swap(face1, face2);
        return 3;
    }
    else return 0; //�������������0
}

void makeAAG()
{
    for(auto ec_it = edge_curves.begin(); ec_it != edge_curves.end(); ec_it++)
    {
        string edgeType = ec_it->second.indexType(ec_it->second.edge); //����������
        if(edgeType != "LINE" && edgeType != "CIRCLE") continue; //��Ѱ�ҹ�������ֱ�߻�Բ��������
        for(int up1 = 0; up1 < ec_it->second.upIndexes.size()-1; up1++) //�����øñ����ߵ����������֮����������
        {
            for(int up2 = up1+1; up2 < ec_it->second.upIndexes.size(); up2++)
            {
                auto oe_it1 = oriented_edges.find(ec_it->second.upIndexes[up1]); //�����1
                int face1 = oe_it1->second.findFace(); //������1
                auto oe_it2 = oriented_edges.find(ec_it->second.upIndexes[up2]); //�����2
                int face2 = oe_it2->second.findFace(); //������2
                int situation = adjacencySituation(face1, face2, ec_it->second.index); //�������(face1�Զ�תΪ����)
                if(situation == 1) {
                    //ƽ���ƽ�����ڣ���������ֱ��

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it1 = planes.find(ad_it1->second.face);
                    auto pl_it2 = planes.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it1->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(pl_it2->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //ƽ��1��Z������
                    Vector dirZ2 = dirZ_it2->second.getVector(); //ƽ��2��Z������
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //ƽ��1���ⷨ����N1
                    Vector N2 = (ad_it2->second.flag == true) ? dirZ2 : toOppositeVector(dirZ2); //ƽ��2���ⷨ����N2

                    auto line_it = lines.find(ec_it->second.edge);
                    auto vec_it = vectors.find(line_it->second.vector);
                    auto dirE_it = directions.find(vec_it->second.direction);
                    Vector dirE = dirE_it->second.getVector();
                    Vector Ne = (oe_it1->second.flag == ec_it->second.flag) ? dirE : toOppositeVector(dirE); //��ƽ��1Ϊ����ʱ�ı�����Ne

                    Vector N = Ne ^ N2; //������ˣ�N������������ƽ��F2�ڴ�ֱ�ڹ�����e��ƽ���ڲ��ķ�������

                    double theta = getAngle(N, N1); // N �� N1 �ļнǦ�
                    int concavity; //��͹��
                    if(fabs(theta) > PI / 2) concavity = 1; // |��| > ��/2ʱΪ͹
                    else if(fabs(theta) < PI / 2) concavity = 0; // |��| < ��/2ʱΪ��
                    else concavity = -1; // |��| = ��/2ʱΪ�ǰ���͹

                    double angleN = getAbsAngle(N1, N2); //��ƽ���ⷨ�����ļнǾ���ֵ
                    double angle;//��ƽ��������н�
                    if(concavity == 1) {
                        angle = angleN + PI;
                    }
                    else if(concavity == 0) {
                        angle = PI - angleN;
                    }
                    else angle = PI;

                    CommonEdge commonEdge(ec_it->second.index, "LINE", concavity, angle); //����ֱ�߱�
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //��ƽ������ڹ�ϵ
                    ad_it2->second.adjacentFaces.emplace(ad_it1->second.index, commonEdge);
                }
                else if(situation == 2) {
                    //ƽ����������ڣ���������ֱ��

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it = planes.find(ad_it1->second.face);
                    auto cy_it = cylindrical_surfaces.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(cy_it->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //ƽ���Z������
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //ƽ����ⷨ����N1

                    auto vp_it = vertex_points.find(ec_it->second.vertex1);
                    auto cp_it1 = cartesian_points.find(vp_it->second.point);
                    Point pt = cp_it1->second.toPoint(); //����ֱ�߱���һ��P
                    auto cp_it2 = cartesian_points.find(ax_it2->second.point);
                    Point begin = cp_it2->second.toPoint(); //���������������
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ2 = dirZ_it2->second.getVector(); //�����Z����������������������
                    Vector perpendicularVec = GetPerpendicular(pt, begin, dirZ2); //��������һ�������������ߵĴ�������
                    Vector N2 = (ad_it2->second.flag == false) ? perpendicularVec : toOppositeVector(perpendicularVec); //�����ڹ���ֱ�߱���һ��P���ⷨ����N2

                    auto line_it = lines.find(ec_it->second.edge);
                    auto vec_it = vectors.find(line_it->second.vector);
                    auto dirE_it = directions.find(vec_it->second.direction);
                    Vector dirE = dirE_it->second.getVector();
                    auto pl_oe_it = (oe_it1->second.findFace() == face1) ? oe_it1 : oe_it2;
                    Vector Ne = (pl_oe_it->second.flag == ec_it->second.flag) ? dirE : toOppositeVector(dirE); //��ƽ��Ϊ����ʱ�ı�����Ne
                    
                    Vector N = Ne ^ N2; //������ˣ�N��������������������ƽ��Ĺ���ֱ�߱��ϵ�һ�����ϣ���ֱ�ڹ����������淽�����������

                    double theta = getAngle(N, N1); // N �� N1 �ļнǦ�
                    int concavity; //��͹��
                    if(fabs(theta) > PI / 2) concavity = 1; // |��| > ��/2ʱΪ͹
                    else if(fabs(theta) < PI / 2) concavity = 0; // |��| < ��/2ʱΪ��
                    else concavity = -1; // |��| = ��/2ʱΪ�ǰ���͹

                    double angleN = getAbsAngle(N1, N2); //ƽ��������ⷨ�����ļнǾ���ֵ
                    double angle;//ƽ�������������н�
                    if(concavity == 1) {
                        angle = angleN + PI;
                    }
                    else if(concavity == 0) {
                        angle = PI - angleN;
                    }
                    else angle = PI;

                    CommonEdge commonEdge(ec_it->second.index, "LINE", concavity, angle); //����ֱ�߱�
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //ƽ�����������ڹ�ϵ
                    ad_it2->second.adjacentFaces.emplace(ad_it1->second.index, commonEdge);
                }
                else if(situation == 3) {
                    //ƽ����������ڣ���������Բ

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it = planes.find(ad_it1->second.face);
                    auto cy_it = cylindrical_surfaces.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(cy_it->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //ƽ���Z������
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //ƽ����ⷨ����N1

                    auto vp_it = vertex_points.find(ec_it->second.vertex1);
                    auto cp_it1 = cartesian_points.find(vp_it->second.point);
                    Point pt = cp_it1->second.toPoint(); //����Բ����һ��P
                    auto cp_it2 = cartesian_points.find(ax_it2->second.point);
                    Point begin = cp_it2->second.toPoint(); //���������������
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ2 = dirZ_it2->second.getVector(); //�����Z����������������������
                    Vector perpendicularVec = GetPerpendicular(pt, begin, dirZ2); //��������һ�������������ߵĴ�������
                    Vector PVec = perpendicularVec; //PVec

                    Vector N2 = (ad_it2->second.flag == false) ? perpendicularVec : toOppositeVector(perpendicularVec); //�����ڹ���Բ����һ��P���ⷨ����N2
                    
                    auto cc_it = circles.find(ec_it->second.edge);
                    auto ax_it3 = axis2_pacement_3ds.find(cc_it->second.axis2);
                    auto dirZ_it3 = directions.find(ax_it3->second.directionZ);
                    Vector dirZ3 = dirZ_it3->second.getVector(); // Բ�����ߵ�Z������
                    Vector Vec = dirZ3; //Vec

                    auto pl_oe_it = (oe_it1->second.findFace() == face1) ? oe_it1 : oe_it2;
                    Vector Ne = (pl_oe_it->second.flag == ec_it->second.flag) ? (PVec ^ Vec) : (Vec ^ PVec); //��ƽ��Ϊ����ʱ����Բ����һ��P�����߱�����Ne

                    Vector N = Ne ^ N2; //������ˣ�N��������������������ƽ��Ĺ���Բ�α��ϵ�һ�����ϣ���ֱ�ڹ����������淽�����������

                    double theta = getAngle(N, N1); // N �� N1 �ļнǦ�
                    int concavity; //��͹��
                    if(fabs(theta) > PI / 2) concavity = 1; // |��| > ��/2ʱΪ͹
                    else if(fabs(theta) < PI / 2) concavity = 0; // |��| < ��/2ʱΪ��

                    double angle;//ƽ�������������н�
                    if(concavity == 1) {
                        angle = PI * 1.5;
                    }
                    else if(concavity == 0) {
                        angle = PI * 0.5;
                    }

                    CommonEdge commonEdge(ec_it->second.index, "CIRCLE", concavity, angle); //����Բ�α�
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //ƽ�����������ڹ�ϵ
                    ad_it2->second.adjacentFaces.emplace(ad_it1->second.index, commonEdge);
                }
            }
        }
    }
}

void cylinder_and_circularHole()
{
    for(auto cy_it = cylindrical_surfaces.begin(); cy_it != cylindrical_surfaces.end(); cy_it++)
    {
        auto ad_it = advanced_faces.find(cy_it->second.upIndex); //�����������ҵ��߼���
        if(ad_it->second.flag == true) {
            //Բ���ж�
            vector<int> o; //Բ�α߽��Բ�ĵѿ�����������
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //�ҵ��߽�
                    int circle_index;
                    if(fob_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //�ҵ��߽�
                    int circle_index;
                    if(fb_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                }
            }
            //Բ���ж�����
            if(o.size() == 2) {
                //���溬������Բ����߽�ʱ��ʶ��ΪԲ�������������Ϣ
                cout << endl << "Բ��Cylinder: radius = " << cy_it->second.radius << ", height = " << pointDistance(o[0], o[1]) << endl;
            }
        }
        else {
            //Բ���ж�
            int n = 0; //�߻�ֻ����һ������ߵı߽�����
            vector<int> o; //Բ�α߽��Բ�ĵѿ�����������
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //�ҵ��߽�
                    int circle_index;
                    if(fob_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��,n++
                        n++;
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                    else if(fob_it->second.isSingleEdge() == true) {  //�߽粻��Բ�����߻�ֻ����һ�������
                        n++;
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //�ҵ��߽�
                    int circle_index;
                    if(fb_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��,n++
                        n++;
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                    else if(fb_it->second.isSingleEdge() == true) {  //�߽粻��Բ�����߻�ֻ����һ�������
                        n++;
                    }
                }
            }
            //Բ���ж�����
            if(o.size() == 2) {
                //���溬������Բ����߽�ʱ��ʶ��Ϊƽ��Բ�ף����������Ϣ
                cout << endl << "ƽ��Բ��Circular hole: radius = " << cy_it->second.radius << ", length = " << pointDistance(o[0], o[1]) << endl;
            }
            else if(o.size() == 1) {
                //���溬��һ��Բ����߽�ʱ��ʶ��ΪԲ�ף����������Ϣ
                cout << endl << "Բ��Circular hole: radius = " << cy_it->second.radius << endl;
            }
            else if(ad_it->second.bounds.size() == 2 && n == 2) {
                //�������ҽ���������߽磬��ÿ����߽�ֻ��һ�������ʱ��ʶ��Ϊ����Բ�ף����������Ϣ
                cout << endl << "����Բ��Curved circular hole: radius = " << cy_it->second.radius << endl;
            }
        }
    }
}

void cone_and_conicalHole()
{
    for(auto co_it = conical_surfaces.begin(); co_it != conical_surfaces.end(); co_it++)
    {
        auto ad_it = advanced_faces.find(co_it->second.upIndex); //�����������ҵ��߼���
        
        //Բ׶Բ̨�ж�����׼��
        int n = 0; //�߻�ֻ����һ������ߵı߽�����
        int v = -1; //����ѿ����������ţ�-1����û��׶���㣩
        vector<int> c; //Բ�α߽��CIRCLE������
        vector<int> o; //Բ�α߽��Բ�ĵѿ�����������
        vector<double> r; //Բ�α߽�İ뾶
        for(int i = 0; i < ad_it->second.bounds.size(); i++)
        {
            int bd_id = ad_it->second.bounds[i];
            if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                auto fob_it = face_outer_bounds.find(bd_id); //�ҵ��߽�
                int vertex_index;
                int circle_index;
                if(fob_it->second.isVERTEX(vertex_index) == true) { //�߽��Ƕ��㣺��¼����ĵѿ�����������
                    auto vp_it = vertex_points.find(vertex_index);
                    v = vp_it->second.point;
                }
                else if(fob_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ+Բ��+�뾶
                    c.emplace_back(circle_index);
                    auto cc_it = circles.find(circle_index);
                    auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                    o.emplace_back(ax_it->second.point);
                    r.emplace_back(cc_it->second.radius);
                }
                else if(fob_it->second.isSingleEdge() == true) { //�߽粻�Ƕ���Ҳ����Բ�����߻�ֻ����һ�������
                    n++;
                }
            }
            if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                auto fb_it = face_bounds.find(bd_id); //�ҵ��߽�
                int vertex_index;
                int circle_index;
                if(fb_it->second.isVERTEX(vertex_index) == true) { //�߽��Ƕ��㣺��¼����ĵѿ�����������
                    auto vp_it = vertex_points.find(vertex_index);
                    v = vp_it->second.point;
                }
                else if(fb_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ+Բ��+�뾶
                    c.emplace_back(circle_index);
                    auto cc_it = circles.find(circle_index);
                    auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                    o.emplace_back(ax_it->second.point);
                    r.emplace_back(cc_it->second.radius);
                }
                else if(fb_it->second.isSingleEdge() == true) { //�߽粻�Ƕ���Ҳ����Բ�����߻�ֻ����һ�������
                    n++;
                }
            }
        }

        //�����ж�
        if(vertex_loops.size() > 0) {
            //�ж����׼
            if(ad_it->second.flag == true) {
                //Բ׶Բ̨���
                if(v != -1 && c.size() == 1) {
                    //Բ׶��׶��+T+��һ��VERTEX_LOOP�߽�+һ��CIRCLE�߽�
                    cout << endl << "Բ׶Cone: ����뾶bottom radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v != -1 && c.size() == 0) {
                    //����Բ׶��׶��+T+��һ��VERTEX_LOOP�߽�+û��CIRCLE�߽�
                    cout << endl << "����Բ׶Curved cone: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 2) {
                    //Բ̨��׶��+T+������CIRCLE�߽�+û��VERTEX_LOOP�߽�
                    cout << endl << "Բ̨Frustum: ��뾶radius1 = " << max(r[0], r[1]) << ", С�뾶radius2 = " << min(r[0], r[1]) << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //������Բ̨��׶��+T+��һ��CIRCLE�߽�+һ�������ϵ��߽�+û��VERTEX_LOOP�߽�
                    cout << endl << "������Բ̨Single curved frustum: �뾶radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 0 && n >= 2) {
                    //˫����Բ̨��׶��+T+�����������ϵ��߽�+û��CIRCLE�߽�+û��VERTEX_LOOP�߽磨����׼������bug���ڣ�
                    cout << endl << "˫����Բ̨Double curved frustum: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
            else {
                //Բ׶��Բ̨�����
                if(v != -1 && c.size() == 1) {
                    //Բ׶�ף�׶��+F+��һ��VERTEX_LOOP�߽�+һ��CIRCLE�߽�
                    cout << endl << "Բ׶��Conical hole: �׾�hole radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v != -1 && c.size() == 0) {
                    //����Բ׶�ף�׶��+F+��һ��VERTEX_LOOP�߽�+û��CIRCLE�߽�
                    cout << endl << "����Բ׶��Curved conical hole: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 2) {
                    //Բ̨�ף�׶��+F+������CIRCLE�߽�+û��VERTEX_LOOP�߽�
                    cout << endl << "Բ̨��Frustum hole: ��뾶radius1 = " << max(r[0], r[1]) << ", С�뾶radius2 = " << min(r[0], r[1]) << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //ƽ������Բ̨�ף�׶��+F+���ҽ���һ��CIRCLE�߽磨͹��+һ�������ϵ��߽�+û��VERTEX_LOOP�߽�
                    cout << endl << "ƽ������Բ̨��Frustum hole with flat surface and curved bottom: �׾�hole radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //����ƽ��Բ̨�ף�׶��+F+���ҽ���һ��CIRCLE�߽磨����+һ�������ϵ��߽�+û��VERTEX_LOOP�߽�
                    cout << endl << "����ƽ��Բ̨��Frustum hole with curved surface and flat bottom: ����뾶bottom radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 0 && n >= 2) {
                    //��������Բ̨�ף�׶��+F+�����������ϵ��߽�+û��CIRCLE�߽�+û��VERTEX_LOOP�߽磨����׼������bug���ڣ�
                    cout << endl << "��������Բ̨��Frustum hole with curved surface and curved bottom: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
        }
        else {
            //�޶����׼
            if(ad_it->second.flag == true) {
                //Բ׶Բ̨���
                if(ad_it->second.bounds.size() == 1 && c.size() == 1) {
                    //Բ׶��׶��+T+���ҽ���һ���߽磬�߽���CIRCLE����׼��
                    cout << endl << "Բ׶Cone: ����뾶bottom radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n == 1 && c.size() == 0) {
                    //����Բ׶��׶��+T+���ҽ���һ�����߽�+û��CIRCLE�߽磨����׼������bug���ڣ�
                    cout << endl << "����Բ׶Curved cone: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n >= 2 && c.size() == 0) {
                    //����Բ׶/˫����Բ̨��׶��+T+�����������ϵ��߽�+û��CIRCLE�߽磨����׼������bug���ڣ�����ģ����
                    cout << endl << "����Բ׶Curved cone/˫����Բ̨Double curved frustum: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 2) {
                    //Բ̨��׶��+T+������CIRCLE�߽磨��׼��
                    cout << endl << "Բ̨Frustum: ��뾶radius1 = " << max(r[0], r[1]) << ", С�뾶radius2 = " << min(r[0], r[1]) << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //Բ׶/������Բ̨��׶��+T+���ҽ���һ��CIRCLE�߽�+һ�������ϵ��߽磨����ģ����
                    cout << endl << "Բ׶Cone/������Բ̨Single curved frustum: �뾶radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
            else {
                //Բ׶��Բ̨�����
                if(ad_it->second.bounds.size() == 1 && c.size() == 1) {
                    //Բ׶�ף�׶��+F+���ҽ���һ���߽磬�߽���CIRCLE����׼��
                    cout << endl << "Բ׶��Conical hole: �׾�hole radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n == 1 && c.size() == 0) {
                    //����Բ׶�ף�׶��+F+���ҽ���һ�����߽�+û��CIRCLE�߽磨����׼������bug���ڣ�
                    cout << endl << "����Բ׶��Curved conical hole: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n >= 2 && c.size() == 0) {
                    //����Բ׶��/��������Բ̨�ף�׶��+F+�����������ϵ��߽�+û��CIRCLE�߽磨����׼������bug���ڣ�����ģ����
                    cout << endl << "����Բ׶��Curved conical hole/��������Բ̨��Frustum hole with curved surface and curved bottom: ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 2) {
                    //Բ̨�ף�׶��+F+������CIRCLE�߽磨��׼��
                    cout << endl << "Բ̨��Frustum hole: ��뾶radius1 = " << max(r[0], r[1]) << ", С�뾶radius2 = " << min(r[0], r[1]) << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //Բ׶��/ƽ������Բ̨�ף�׶��+F+���ҽ���һ��CIRCLE�߽磨͹��+һ�������ϵ��߽磨����ģ����
                    cout << endl << "Բ׶��Conical hole/ƽ������Բ̨��Frustum hole with flat surface and curved bottom: �׾�hole radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //����ƽ��Բ̨�ף�׶��+F+���ҽ���һ��CIRCLE�߽磨����+һ�������ϵ��߽磨����׼������bug���ڣ�����ģ����
                    cout << endl << "����ƽ��Բ̨��Frustum hole with curved surface and flat bottom: ����뾶bottom radius = " << r[0] << ", ��׶��coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
        }
    }
}


bool setUse(const int &n)
{
    if(n != 0 && n != 1 && n != 2) return false;
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        ad_it->second.isUsed = n;
    }
    return true;
}

bool setVisit(const int &n)
{
    if(n != 0 && n != 1) return false;
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        ad_it->second.isVisited = n;
    }
    return true;
}

void closedPocket()
{
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        if(ad_it->second.faceType != "PLANE") continue; //����ƽ��ֱ������

        if(ad_it->second.concaveCommonLineEdge_number() < 4) continue; //������ֱ�߱߸����������4��
        //cout << "Plane" << ad_it->first<<endl;

        //�ڵ��������ҵ�һ���ߴ�������4��������ֱ�߱���ɵıջ��߽磨�Ƚϱ߳�����Ϊ�˹�ܵ����ڲ����ܳ��ֵ���������
        vector<map<int, CommonEdge>::iterator> concaveAdjacentFaces; //���ջ��߽簼������ĵ�����
        if(ad_it->second.get_4concaveAdjacentFaces_closedLoopOutside(concaveAdjacentFaces) == false) continue;
        //cout << "curve " << concaveAdjacentFaces[0]->second.index << ' ' << concaveAdjacentFaces[1]->second.index << ' ' << concaveAdjacentFaces[2]->second.index << ' ' << concaveAdjacentFaces[3]->second.index << ' ' << endl;

        //4������������붼��ƽ��
        vector<map<int, ADVANCED_FACE>::iterator> faces; //������߼���ĵ�����
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[0]->first));
        if(faces[0]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[1]->first));
        if(faces[1]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[2]->first));
        if(faces[2]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[3]->first));
        if(faces[3]->second.faceType != "PLANE") continue;
        // map<int, CommonEdge>::iterator it = ad_it->second.adjacentFaces.begin(); //map<������������, ����������>������
        // int index1 = it->first; //4���������������
        // map<int, PLANE>::iterator plane1 = planes.find(index1); //4��������
        // if(plane1 == planes.end()) continue; //4����������붼��ƽ��
        // map<int, CommonEdge>::iterator commonEdge1 = it; //ÿ���������Ӧ�Ĺ�����
        // it++;
        // int index2 = it->first;
        // map<int, PLANE>::iterator plane2 = planes.find(index2);
        // if(plane2 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge2 = it;
        // it++;
        // int index3 = it->first;
        // map<int, PLANE>::iterator plane3 = planes.find(index3);
        // if(plane3 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge3 = it; //ÿ���������Ӧ�Ĺ�����
        // it++;
        // int index4 = it->first;
        // map<int, PLANE>::iterator plane4 = planes.find(index4);
        // if(plane4 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge4 = it; //ÿ���������Ӧ�Ĺ�����
        // it++;

        //�밼������İ��߱߼нǱ���Ϊ��/2��90�ȣ�
        if(!anglesAreEqual(concaveAdjacentFaces[0]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[1]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[2]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[3]->second.angle, (PI/2))) continue;

        //�ĸ���������֮�����ҽ����ĸ�����ֱ�߱ߣ���ÿ������������֮����һ������������ĸ�����ֱ�߱�
        vector<CommonEdge*> sideCommonEdges; //�ĸ�������������֮��Ĺ�����ָ��
        if(reorderAdjacentFaces(concaveAdjacentFaces, faces, sideCommonEdges) == false) continue; //�������ĸ���������

        //���ĸ������߶��ǰ����Ҽн�Ϊ��/2��90�ȣ�
        if(sideCommonEdges[0]->concavity != 0 || !anglesAreEqual(sideCommonEdges[0]->angle, (PI/2))) continue;
        if(sideCommonEdges[1]->concavity != 0 || !anglesAreEqual(sideCommonEdges[1]->angle, (PI/2))) continue;
        if(sideCommonEdges[2]->concavity != 0 || !anglesAreEqual(sideCommonEdges[2]->angle, (PI/2))) continue;
        if(sideCommonEdges[3]->concavity != 0 || !anglesAreEqual(sideCommonEdges[3]->angle, (PI/2))) continue;

        //���ĸ�����������洹ֱ���һ���֮��ƽ��
        vector<Vector> sideVec; //�ĸ�����ֱ�߱ߵ�����
        for(int i = 0; i < 4; i++)
        {
            sideVec.emplace_back(sideCommonEdges[i]->getVector());
        }
        Vector bottomFaceVec = ad_it->second.getDirectionZ(); //����Z������
        if(isParallel(sideVec[0], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[1], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[2], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[3], bottomFaceVec) == false) continue;
        if(allParallel(sideVec) == false) continue;
        //cout << "All Parallel" << endl;

        /*�ڲ���׼�ж���
        �ڲ�����涼ֻ���б�Ҫ�ߣ�����һ���ջ��߽磩ʱ�ж�Ϊ�ڲ���׼������Ϊ�ڲ��Ǳ�׼
        ����Ҫ��Ϊ���������������û�п���͹������⴦��
        */
        bool inside_typical = true; //�ڲ��Ƿ��׼
        if(ad_it->second.bounds.size() != 1) inside_typical = false; //����ջ��߽粻��һ�����ڲ��Ǳ�׼
        //cout << "inside bounds.size = " << ad_it->second.bounds.size() << endl;
        for(int i = 0; i < 4; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //����ջ��߽粻��һ�����ڲ��Ǳ�׼
            //cout << "outside[" << i << "].bounds.size = " << faces[i]->second.bounds.size() << endl;
        }

        /*�ⲿ��׼�ж���
        �ĸ�����ͬʱ��ͬһ��ƽ�棨���˵��棩������һ��90��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻��������������������ڴ�ֱ�������ⲿ��׼���������ǻ���
        �����ⲿ�Ǳ�׼��
        */
        bool outside_typical = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //�ĸ��������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0���������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //�н�Ϊ90��
            if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //����������ƽ��
                outsideAdjacentFaces.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces.size() == 1) { //���ڵ�0�����棬���������������������ҽ���һ����������ж�
            //cout << "outsideFace = " << outsideAdjacentFaces[0]->first << ", outsideEdge[0] = " << outsideAdjacentFaces[0]->second.index << endl;
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //����ʣ��3���������������
                {
                    if(af_it->first != outsideAdjacentFaces[0]->first) continue; //��i������������������0�����������������ͬһ����
                    if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //�н�Ϊ90��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces.emplace_back(af_it);
                    found = true;
                    //cout << "outsideEdge[" << i << "] = " << af_it->second.index << endl;
                    break;
                }
                if(found == false) break; //��i��������δ�ҵ����0����������������ͬ����������
            }
            //����������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //���������ⲿ��׼
        }
        
        //��ȷ��Ϊ�ⲿ��׼��ǻ�����㼸�β���
        auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //�������湫���ߵı���������
        double depth = sideEdge_it->second.length(); //��ǻ�����
        auto bottomEdge_it1 = edge_curves.find(concaveAdjacentFaces[0]->second.index); //������������ڱ߽�
        auto bottomEdge_it2 = edge_curves.find(concaveAdjacentFaces[2]->second.index);
        double length = bottomEdge_it1->second.length(); //���泤��
        double width = bottomEdge_it2->second.length(); //������
        if(length < width) swap(length, width);

        //�ж���ϣ����
        cout << endl << "��ǻClosed pocket ";
        if(inside_typical == true) cout << "�ڲ���׼Typical inside ";
        else cout << "�ڲ��Ǳ�׼Not typical inside ";
        if(outside_typical == true) cout << "�ⲿ��׼Typical outside: ";
        else cout << "�ⲿ�Ǳ�׼Not typical outside: ";
        if(outside_typical == true) cout << "���depth = " << depth << ", ";
        cout << "���泤��length = " << length << ", ������width = " << width;
        cout << " | ����#" << ad_it->first << ", �ĸ�����#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << endl;

        //���ιؼ���
        ad_it->second.setUse(2); //����Ϊ�ؼ���
        faces[0]->second.setUse(1); //���汻ʶ��Ϊ����
        faces[1]->second.setUse(1);
        faces[2]->second.setUse(1);
        faces[3]->second.setUse(1);
    }
}
bool reorderAdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges)
{
    //�ȴ��Ե������������˳�򣬰����0����û�й����ߵ����ᵽ��1λ
    if(faces[0]->second.commonEdge_number_withFace(faces[2]->first) == 0) {
        swap(concaveAdjacentFaces[1], concaveAdjacentFaces[2]);
        swap(faces[1], faces[2]);
    }
    else if(faces[0]->second.commonEdge_number_withFace(faces[3]->first) == 0) {
        swap(concaveAdjacentFaces[1], concaveAdjacentFaces[3]);
        swap(faces[1], faces[3]);
    }
    //ȷ�������ڵĶ�λ��֮��û�й����ߣ���0���桢��1����֮�䣬��2���桢��3����֮�䣩
    if(faces[0]->second.commonEdge_number_withFace(faces[1]->first) != 0) return false;
    if(faces[2]->second.commonEdge_number_withFace(faces[3]->first) != 0) return false;

    //������������֮�����ҽ���һ������ֱ�߱�
    if(faces[0]->second.commonEdge_number_withFace(faces[2]->first) != 1) return false;
    sideCommonEdges.emplace_back(faces[0]->second.getCommonEdge_withAdjacentFace(faces[2]->first));
    if(faces[0]->second.commonEdge_number_withFace(faces[3]->first) != 1) return false;
    sideCommonEdges.emplace_back(faces[0]->second.getCommonEdge_withAdjacentFace(faces[3]->first));
    if(faces[1]->second.commonEdge_number_withFace(faces[2]->first) != 1) return false;
    sideCommonEdges.emplace_back(faces[1]->second.getCommonEdge_withAdjacentFace(faces[2]->first));
    if(faces[1]->second.commonEdge_number_withFace(faces[3]->first) != 1) return false;
    sideCommonEdges.emplace_back(faces[1]->second.getCommonEdge_withAdjacentFace(faces[3]->first));
    // cout << "side = " << sideCommonEdges[0]->index << " angle" << sideCommonEdges[0]->angle << ' ' << sideCommonEdges[0]->concavity << ' ';
    // cout << "side = " << sideCommonEdges[1]->index << " angle" << sideCommonEdges[1]->angle << ' ' << sideCommonEdges[1]->concavity << ' ';
    // cout << "side = " << sideCommonEdges[2]->index << " angle" << sideCommonEdges[2]->angle << ' ' << sideCommonEdges[2]->concavity << ' ';
    // cout << "side = " << sideCommonEdges[3]->index << " angle" << sideCommonEdges[3]->angle << ' ' << sideCommonEdges[3]->concavity << ' ' << endl;

    // cout << concaveAdjacentFaces[0]->first << concaveAdjacentFaces[1]->first << concaveAdjacentFaces[2]->first << concaveAdjacentFaces[3]->first << endl;
    // cout << faces[0]->first << faces[1]->first << faces[2]->first << faces[3]->first << endl;


    //��֤���ĸ�������������ڹ�ϵ������������������͹���ֱ�߱ߵĻ�ȡ�󣬷��سɹ�
    return true;
}
bool allParallel(const vector<Vector> &vecs)
{
    for(int i = 0; i < vecs.size() - 1; i++)
    {
        for(int j = i + 1; j < vecs.size(); j++)
        {
            if(isParallel(vecs[i], vecs[j]) == false) return false;
        }
    }
    return true;
}
bool formAnEdgeLoop(const vector<map<int, CommonEdge>::iterator> &outsideAdjacentFaces)
{
    bool isLoop = false; //һ�鹫�����Ƿ����ڲ����һ���߻�
    for(auto el_it = edge_loops.begin(); el_it != edge_loops.end(); el_it++)
    {
        if(el_it->second.edges.size() != outsideAdjacentFaces.size()) continue; //�ñ߻����б߸����빫���߸�����ͬ
        bool belong = false; //�����߻��е�ÿ�����Ƿ������鹫����֮һ
        for(int i = 0; i < el_it->second.edges.size(); i++)
        {
            belong = false;
            auto oe_it = oriented_edges.find(el_it->second.edges[i]);
            auto ec_it = edge_curves.find(oe_it->second.curve);
            for(int j = 0; j < outsideAdjacentFaces.size(); j++)
            {
                if(ec_it->first == outsideAdjacentFaces[j]->second.index) {
                    belong = true;
                    break;
                }
            }
            if(belong == false) break;
        }
        if(belong == true) {
            isLoop = true;
            break;
        }
    }
    return isLoop;
}

void hexagonalClosedPocket()
{
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        if(ad_it->second.faceType != "PLANE") continue; //����ƽ��ֱ������

        if(ad_it->second.concaveCommonLineEdge_number() < 6) continue; //������ֱ�߱߸����������6��

        //�ڵ��������ҵ�һ���ߴ�������6��ͬ�����ȵİ�����ֱ�߱���ɵıջ��߽磨�Ƚϱ߳�����Ϊ�˹�ܵ����ڲ����ܳ��ֵ���������
        vector<map<int, CommonEdge>::iterator> concaveAdjacentFaces; //���ջ��߽簼������ĵ�����
        if(ad_it->second.get_6concaveAdjacentFaces_closedLoopOutside(concaveAdjacentFaces) == false) continue;

        //6������������붼��ƽ��
        bool allPlane = true;
        vector<map<int, ADVANCED_FACE>::iterator> faces; //������߼���ĵ�����
        for(int i = 0; i < concaveAdjacentFaces.size(); i++)
        {
            faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[i]->first));
            if(faces[i]->second.faceType != "PLANE") {
                allPlane = false;
                break;
            }
        }
        if(allPlane == false) continue;

        //�밼������İ��߱߼нǱ���Ϊ��/2��90�ȣ�
        bool allVertical = true;
        for(int i = 0; i < concaveAdjacentFaces.size(); i++)
        {
            if(!anglesAreEqual(concaveAdjacentFaces[i]->second.angle, (PI/2))) {
                allVertical = false;
                break;
            }
        }
        if(allVertical == false) continue;

        //������������֮�����ҽ�����������ֱ�߱ߣ���ÿ������������֮����һ�������������������ֱ�߱�
        vector<CommonEdge*> sideCommonEdges; //����������������֮��Ĺ�����ָ��
        if(reorder6AdjacentFaces(concaveAdjacentFaces, faces, sideCommonEdges) == false) continue; //������������������

        //�����������߶��ǰ����Ҽн�Ϊ2��/3��120�ȣ���
        bool allConcaveAngle120 = true;
        for(int i = 0; i < 6; i++)
        {
            if(sideCommonEdges[i]->concavity != 0 || !anglesAreEqual(sideCommonEdges[i]->angle, (2*PI/3))) {
                allConcaveAngle120 = false;
                break;
            }
        }
        if(allConcaveAngle120 == false) continue;

        //����������������洹ֱ���һ���֮��ƽ�С�
        bool areAllParallel = true;
        Vector bottomFaceVec = ad_it->second.getDirectionZ(); //����Z������
        vector<Vector> sideVec; //��������ֱ�߱ߵ�����
        for(int i = 0; i < 6; i++)
        {
            sideVec.emplace_back(sideCommonEdges[i]->getVector());
            if(isParallel(sideVec[i], bottomFaceVec) == false) {
                areAllParallel = false;
                break;
            }
        }
        if(areAllParallel == false) continue;
        if(allParallel(sideVec) == false) continue;

        /*�ڲ���׼�ж���
        �ڲ��߸��涼ֻ���б�Ҫ�ߣ�����һ���ջ��߽磩ʱ�ж�Ϊ�ڲ���׼������Ϊ�ڲ��Ǳ�׼
        ����Ҫ��Ϊ�������߸�������û�п���͹������⴦��
        */
        bool inside_typical = true; //�ڲ��Ƿ��׼
        if(ad_it->second.bounds.size() != 1) inside_typical = false; //����ջ��߽粻��һ�����ڲ��Ǳ�׼
        for(int i = 0; i < 6; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //����ջ��߽粻��һ�����ڲ��Ǳ�׼
        }

        /*�ⲿ��׼�ж���
        ��������ͬʱ��ͬһ��ƽ�棨���˵��棩������һ��90��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻��������������������ڴ�ֱ�������ⲿ��׼���������ǻ���
        �����ⲿ�Ǳ�׼��
        */
        bool outside_typical = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //�����������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0���������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //�н�Ϊ90��
            if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //����������ƽ��
                outsideAdjacentFaces.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces.size() == 1) { //���ڵ�0�����棬���������������������ҽ���һ����������ж�
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //����ʣ��5���������������
                {
                    if(af_it->first != outsideAdjacentFaces[0]->first) continue; //��i������������������0�����������������ͬһ����
                    if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //�н�Ϊ90��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //��i��������δ�ҵ����0����������������ͬ����������
            }
            //����������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //���������ⲿ��׼

            //��ȷ��Ϊ�ⲿ��׼������ǻ�����㼸�β���
            auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //�������湫���ߵı���������
            double depth = sideEdge_it->second.length(); //������ǻ�����
            auto bottomEdge_it = edge_curves.find(concaveAdjacentFaces[0]->second.index); //�����һ�����ڱ߽�
            double length = bottomEdge_it->second.length(); //����߳�

            //�ж���ϣ����
            cout << endl << "������ǻHexagonal Closed pocket ";
            if(inside_typical == true) cout << "�ڲ���׼Typical inside ";
            else cout << "�ڲ��Ǳ�׼Not typical inside ";
            if(outside_typical == true) cout << "�ⲿ��׼Typical outside: ";
            else cout << "�ⲿ�Ǳ�׼Not typical outside: ";
            if(outside_typical == true) cout << "���depth = " << depth << ", ";
            cout << "����߳�length = " << length;
            cout << " | ����#" << ad_it->first << ", ��������#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

            //���ιؼ���
            ad_it->second.setUse(2);
            for(int i = 0; i < 6; i++)
            {
                faces[i]->second.setUse(1);
            }
        }
    }
}
bool reorder6AdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges)
{
    //�������������水˳ʱ�����ʱ��˳���������кã�����֤ÿ�������������ҽ���һ������ֱ�߱�
    for(int i = 0; i < 4; i++)
    {
        bool found = false; //�Ƿ��ҵ����i�������ҽ���һ������ֱ�߱ߵ���
        for(int j = i + 1; j < 6; j++)
        {
            if(faces[i]->second.commonEdge_number_withFace(faces[j]->first) == 1) {
                swap(concaveAdjacentFaces[i + 1], concaveAdjacentFaces[j]);
                swap(faces[i + 1], faces[j]);
                found = true;
                break;
            }
        }
        if(found == false) return false; //û���ҵ������棬����ʧ��
    }
    if(faces[5]->second.commonEdge_number_withFace(faces[4]->first) != 1) return false;
    if(faces[5]->second.commonEdge_number_withFace(faces[0]->first) != 1) return false;

    //ȷ�������ڵĶ�λ��֮��û�й�����
    for(int i = 0; i < 6; i++)
    {
        for(int j = 2; j <= 4; j++)
        {
            if(faces[i]->second.commonEdge_number_withFace(faces[(i + j) % 6]->first) != 0) return false; //�������������������й����ߣ����ش���
        }
    }

    //���η�����������ֱ�߱�ָ��
    for(int i = 0; i < 6; i++)
    {
        sideCommonEdges.emplace_back(faces[i]->second.getCommonEdge_withAdjacentFace(faces[(i + 1) % 6]->first));
    }

    //��֤������������������ڹ�ϵ������������������͹���ֱ�߱ߵĻ�ȡ�󣬷��سɹ�
    return true;
}