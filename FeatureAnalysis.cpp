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
                    
                    //cout << "�����1��#" << oe_it1->first << ",�����2��#" << oe_it2->first << ",�����ߣ�#" << ec_it->first << "��Ne: " << Ne.show() << ", N1: " << N1.show() << ", N2: " << N2.show() << ", N: " << N.show() << endl;
                    
                    double theta = getAngle(N, N1); // N �� N1 �ļнǦ�
                    
                    //cout << "�� = " << theta << endl;

                    int concavity; //��͹��
                    if(fabs(theta) > (PI / 2.0)) concavity = 1; // |��| > ��/2ʱΪ͹
                    else if(fabs(theta) < (PI / 2.0)) concavity = 0; // |��| < ��/2ʱΪ��
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

        if(ad_it->second.concaveCommonLineEdge_number() < 4) continue; //������ֱ�߱߸������벻����4��
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
        �ĸ�����ͬʱ��ͬһ��ƽ�棨���˵��棩������һ��270��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻����������������������ڴ�ֱ���������ⲿ��׼���������ǻ���
        �����ⲿ�Ǳ�׼��
        */
        bool outside_typical = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //�ĸ��������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0���������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
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
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
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
        else if(outsideAdjacentFaces.size() > 1) { // ��0�������ҵ����������������������1����Ҫ���λ�ĵ�1������ȶԹ�ͬ�������������ĸ�,�ټ����ж�
            bool foundCommonFace = false; // �Ƿ��ҵ���0�������1����Ĺ�����������
            for(auto af_it = faces[1]->second.adjacentFaces.begin(); af_it != faces[1]->second.adjacentFaces.end(); af_it++) //�ҵ����0�������λ�ĵ�1��������������棬�Աȵõ�������������
            {
                if(af_it->second.concavity != 1) continue; //͹
                if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                for(auto oaf_itt = outsideAdjacentFaces.begin(); oaf_itt != outsideAdjacentFaces.end(); oaf_itt++)
                {
                    if(af_it->first == (*oaf_itt)->first) { //�ҵ�������������
                        outsideAdjacentFaces.clear();
                        outsideAdjacentFaces.emplace_back(*oaf_itt);
                        outsideAdjacentFaces.emplace_back(af_it);
                        foundCommonFace = true;
                        break;
                    }
                }
                if(foundCommonFace == true) break; //�ҵ��˹�����������
            }
            //���2���桢��3��������жϹ�����������
            if(foundCommonFace == true) {
                for(int i = 2; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //����ʣ��2���������������
                    {
                        if(af_it->first != outsideAdjacentFaces[0]->first) continue; //��i������������������0�����������������ͬһ����
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i��������δ�ҵ����0����������������ͬ����������
                }
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //���������ⲿ��׼
            }
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

        // if(ad_it->second.bounds.size() == 1) {
        //     auto fob_it = face_outer_bounds.find(ad_it->second.bounds[0]);
        //     if(fob_it != face_outer_bounds.end()) {
        //         auto el_it = edge_loops.find(fob_it->second.loop);
        //         if(el_it->second.edges.size() == 6) {
        //             cout << "face = " << ad_it->first << ", edge = ";
        //             for(int i = 0; i < 6; i++)
        //             {
        //                 auto oe_it = oriented_edges.find(el_it->second.edges[i]);
        //                 auto ec_it = edge_curves.find(oe_it->second.curve);
        //                 cout << ec_it->first << ' ';
        //             }
        //             cout << endl;
        //         }
        //     }
        // }
        if(ad_it->second.concaveCommonLineEdge_number() < 6) continue; //������ֱ�߱߸������벻����6��
        //cout << "face" << ad_it->first << endl;

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
        ��������ͬʱ��ͬһ��ƽ�棨���˵��棩������һ��270��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻����������������������ڴ�ֱ���������ⲿ��׼���������ǻ���
        �����ⲿ�Ǳ�׼��
        */
        bool outside_typical = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //�����������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0���������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
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
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
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

            // //��ȷ��Ϊ�ⲿ��׼������ǻ�����㼸�β���
            // auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //�������湫���ߵı���������
            // double depth = sideEdge_it->second.length(); //������ǻ�����
            // auto bottomEdge_it = edge_curves.find(concaveAdjacentFaces[0]->second.index); //�����һ�����ڱ߽�
            // double length = bottomEdge_it->second.length(); //����߳�

            // //�ж���ϣ����
            // cout << endl << "������ǻHexagonal Closed pocket ";
            // if(inside_typical == true) cout << "�ڲ���׼Typical inside ";
            // else cout << "�ڲ��Ǳ�׼Not typical inside ";
            // if(outside_typical == true) cout << "�ⲿ��׼Typical outside: ";
            // else cout << "�ⲿ�Ǳ�׼Not typical outside: ";
            // if(outside_typical == true) cout << "���depth = " << depth << ", ";
            // cout << "����߳�length = " << length;
            // cout << " | ����#" << ad_it->first << ", ��������#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

            // //���ιؼ���
            // ad_it->second.setUse(2);
            // for(int i = 0; i < 6; i++)
            // {
            //     faces[i]->second.setUse(1);
            // }
        }
        else if(outsideAdjacentFaces.size() > 1) { // ��0�������ҵ����������������������1����Ҫ�벻���ڵĵ�2������ȶԹ�ͬ�������������ĸ�,�ټ����ж�
            bool foundCommonFace = false; //�Ƿ��ҵ������ڵĵ�2���������0�������й�ͬ����������
            for(auto af_it = faces[2]->second.adjacentFaces.begin(); af_it != faces[2]->second.adjacentFaces.end(); af_it++) //�ҵ����0�����治���ڵĵ�2��������������棬�Աȵõ�������������
            {
                if(af_it->second.concavity != 1) continue; //͹
                if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                for(auto oaf_itt = outsideAdjacentFaces.begin(); oaf_itt != outsideAdjacentFaces.end(); oaf_itt++)
                {
                    if(af_it->first == (*oaf_itt)->first) { //�ҵ�������������
                        outsideAdjacentFaces.clear();
                        outsideAdjacentFaces.emplace_back(*oaf_itt);
                        outsideAdjacentFaces.emplace_back(af_it);
                        foundCommonFace = true;
                        break;
                    }
                }
                if(foundCommonFace == true) break; //�ҵ��˹�����������
            }
            if(foundCommonFace == true) { //�ҵ��˹����������棬�����ж�
                for(int i = 1; i < 6; i++)
                {
                    if(i == 2) continue; //������2������(ǰ���Ѿ��жϹ���)
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //����ʣ��4���������������
                    {
                        if(af_it->first != outsideAdjacentFaces[0]->first) continue; //��i������������������0�����������������ͬһ����
                        //cout << '#' << faces[i]->first << " �� #" << af_it->first << " �н�Ϊ" << af_it->second.angle << "�� " << af_it->second.concavity << endl;
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i��������δ�ҵ����0����������������ͬ����������
                }
                if(outsideAdjacentFaces.size() == 6) swap(outsideAdjacentFaces[1], outsideAdjacentFaces[2]); //����1������������������2��������������潻�����������������水����˳������
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //���������ⲿ��׼
            }
        }

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


void squareHole()
{
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        if(ad_it->second.faceType != "PLANE") continue; //����ƽ��ֱ������

        if(ad_it->second.concaveCommonLineEdge_number() < 2) continue; //������ֱ�߱߸������벻����2��

        //���ҵ�һ�Լн�90�ȵİ�����ֱ�߱ߣ���Ӧ�������������棨ƽ�棩���ҵ���һ����ͬ�İ������棨ƽ�棩����һ�Լн�90�ȵİ�����ֱ�߱ߣ�������Ҫ������ֱ�߱��໥ƽ�У�ÿ����������֮�����ҽ���һ������ֱ�߱�
        vector<map<int, ADVANCED_FACE>::iterator> faces; //�����ĸ���߼���ĵ���������0������1����Ϊһ�Բ����ڵĶ�λ�棬��2������3����Ϊ��һ�Բ����ڵĶ�λ�棩
        vector<CommonEdge*> sideCommonEdges; //�ĸ�������֮��Ĺ�����ָ�루˳��Ϊ����0������2����֮��Ĺ����ߣ���0������3����֮��Ĺ����ߣ���1������2����֮��Ĺ����ߣ���1������3����֮��Ĺ����ߣ�
        bool found = false; //�Ƿ��ҵ������������ĸ���
        for(auto af_it1 = ad_it->second.adjacentFaces.begin(); af_it1 != ad_it->second.adjacentFaces.end(); af_it1++)
        {
            if(advanced_faces.find(af_it1->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
            if(af_it1->second.edgeType != "LINE") continue; //ֱ�߹�����
            if(af_it1->second.concavity != 0) continue; //������ֱ�߱�
            if(!anglesAreEqual(af_it1->second.angle, (PI/2))) continue; //�н�Ϊ90��

            for(auto af_it2 = af_it1; af_it2 != ad_it->second.adjacentFaces.end(); af_it2++)
            {
                if(af_it1->first == af_it2->first) continue; //�����������治����ͬһ����
                if(advanced_faces.find(af_it2->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                if(af_it2->second.edgeType != "LINE") continue; //ֱ�߹�����
                if(af_it2->second.concavity != 0) continue; //������ֱ�߱�
                if(!anglesAreEqual(af_it2->second.angle, (PI/2))) continue; //�н�Ϊ90��
                
                if(!isParallel(af_it1->second.getVector(), af_it2->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                if(ad_it->second.commonEdge_number_withFace(af_it1->first) != 1) continue; //��������������ֱ����ҽ���һ������ֱ�߱�
                if(ad_it->second.commonEdge_number_withFace(af_it2->first) != 1) continue;

                auto afAd_it1 = advanced_faces.find(af_it1->first); //��һ����������ĸ߼��������
                auto afAd_it2 = advanced_faces.find(af_it2->first); //�ڶ�����������ĸ߼��������
                for(auto af_it3 = afAd_it1->second.adjacentFaces.begin(); af_it3 != afAd_it1->second.adjacentFaces.end(); af_it3++)
                {
                    if(advanced_faces.find(af_it3->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                    if(af_it3->second.edgeType != "LINE") continue; //ֱ�߹�����
                    if(af_it3->second.concavity != 0) continue; //������ֱ�߱�
                    if(!anglesAreEqual(af_it3->second.angle, (PI/2))) continue; //�н�Ϊ90��
                    for(auto af_it4 = afAd_it2->second.adjacentFaces.begin(); af_it4 != afAd_it2->second.adjacentFaces.end(); af_it4++)
                    {
                        if(af_it3->first != af_it4->first) continue; //�����������湲ͬ��������һ����
                        if(af_it3->first == ad_it->first) continue; //��һ����ͬ�����治���ǵ�һ����ͬ������
                        if(advanced_faces.find(af_it4->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                        if(af_it4->second.edgeType != "LINE") continue; //ֱ�߹�����
                        if(af_it4->second.concavity != 0) continue; //������ֱ�߱�
                        if(!anglesAreEqual(af_it4->second.angle, (PI/2))) continue; //�н�Ϊ90��

                        if(!isParallel(af_it3->second.getVector(), af_it4->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                        if(afAd_it1->second.commonEdge_number_withFace(af_it3->first) != 1) continue; //��������������ֱ����ҽ���һ������ֱ�߱�
                        if(afAd_it2->second.commonEdge_number_withFace(af_it4->first) != 1) continue;

                        if(!isParallel(af_it1->second.getVector(), af_it3->second.getVector())) continue; //��֤����ֱ�߱�����ƽ��

                        //���淽���ĸ���ĸ߼��������
                        faces.emplace_back(ad_it);
                        auto afAd_it3 = advanced_faces.find(af_it3->first);
                        faces.emplace_back(afAd_it3);
                        faces.emplace_back(afAd_it1);
                        faces.emplace_back(afAd_it2);
                        
                        //�����ĸ�������֮��Ĺ�����ָ��
                        sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it1->first));
                        sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it2->first));
                        sideCommonEdges.emplace_back(afAd_it1->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                        sideCommonEdges.emplace_back(afAd_it2->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                        found = true; //����Ѿ��ҵ��˷����������ĸ���
                        break;
                    }
                    if(found == true) break;
                }
                if(found == true) break;
            }
            if(found == true) break;
        }
        if(found == false) continue; //û���ҵ������������ĸ��棬����

        //�ĸ��治��ȫ���ѱ�ʶ��Ϊ�������棨isUsed == 1����Ҳ�����б����ι��Ĺؼ��棨isUsed == 2��
        bool used = true;
        for(int i = 0; i < 4; i++)
        {
            if(faces[i]->second.isUsed == 2) {
                used = true;
                break;
            }
            else if(faces[i]->second.isUsed == 0) used = false;
        }
        if(used == true) continue;

        /*�ڲ���׼�ж���
        �ڲ��ĸ��涼ֻ���б�Ҫ�ߣ�����һ���ջ��߽磩ʱ�ж�Ϊ�ڲ���׼������Ϊ�ڲ��Ǳ�׼��
        ����Ҫ��Ϊ�������ĸ�������û�п���͹������⴦��
        */
        bool inside_typical = true; //�ڲ��Ƿ��׼
        for(int i = 0; i < 4; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //�ĸ���ջ��߽粻��һ�����ڲ��Ǳ�׼
        }

        /*�ⲿ��׼�ж���
        �ĸ���ͬʱ��ͬһ��ƽ��������һ��270��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻���������
        �������������Ĺ��������������ҵ�����ʱ��Ϊ˫ƽ���ⲿ��׼���ף��������������ȣ������Ĺ�������������ֻ�ҵ�һ��ʱ��Ϊ��ƽ���ⲿ��׼���ף��������������ȡ�
        �����ⲿ�Ǳ�׼��        
        */
        bool outside_typical1 = false, outside_typical2 = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces1; //�ĸ���ĵ�һ��������������
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces2; //�ĸ���ĵڶ���������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0�������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
            if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //����������ƽ��
                outsideAdjacentFaces1.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces1.size() == 2) { //���ڵ�0�����棬����������������������������֤�ĸ�����ֱ�ͬ��������������������
            outsideAdjacentFaces2.emplace_back(outsideAdjacentFaces1[1]);
            outsideAdjacentFaces1.erase(outsideAdjacentFaces1.begin() + 1);
            //����֤��һ��������������
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��3�����Ƿ������������������
                {
                    if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //��i����������������0����ĵ�һ������������ͬһ����
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces1.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //��i������δ�ҵ����0�����һ������������ͬ����������
            }
            //��һ���������������������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces1.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //��һ�������������������ⲿ��׼

            //����֤�ڶ���������������
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��3�����Ƿ������������������
                {
                    if(af_it->first != outsideAdjacentFaces2[0]->first) continue; //��i����������������0����ĵڶ�������������ͬһ����
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces2.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //��i������δ�ҵ����0����ڶ�������������ͬ����������
            }
            //�ڶ����������������������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces2.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces2)) outside_typical2 = true; //�ڶ��������������������ⲿ��׼

            if(outside_typical2 == true && outside_typical1 == false) { //�ڶ���������������ⲿ��׼����һ���������治�����ⲿ��׼������������������
                swap(outsideAdjacentFaces1, outsideAdjacentFaces2);
                swap(outside_typical1, outside_typical2);
            }
        }
        else if(outsideAdjacentFaces1.size() > 2) { //���ڵ�0�����棬�����������������������������֤�ĸ�����ֱ�ͬ���������е�������������
            int num = outsideAdjacentFaces1.size(); //��0������������������
            swap(outsideAdjacentFaces1, outsideAdjacentFaces2); //����ѡ�Ĺ��������������outsideAdjacentFaces2�б���

            //�ҵ�һ��������������
            while(outside_typical1 != true && num > 0) {
                outsideAdjacentFaces1.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;
                
                for(int i = 1; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��3�����Ƿ������������������
                    {
                        if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //��i����������������0������������������ͬһ����
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces1.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i������δ�ҵ����0������������������ͬ����������
                }
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces1.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //�������������������ⲿ��׼
                else outsideAdjacentFaces1.clear();
            }
            
            //�ҵ��˵�һ�������������������棬�����ҵڶ���������������������
            while(outside_typical2 != true && num > 0) {
                vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces3; //��ʱ��������洢��
                outsideAdjacentFaces3.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;

                for(int i = 1; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��3�����Ƿ������������������
                    {
                        if(af_it->first != outsideAdjacentFaces3[0]->first) continue; //��i����������������0������������������ͬһ����
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces3.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i������δ�ҵ����0������������������ͬ����������
                }
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces3.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces3)) { //�������������������ⲿ��׼
                    outside_typical2 = true;
                    outsideAdjacentFaces2 = outsideAdjacentFaces3;
                }
            }
        }

        //���㼸�β���
        double depth; //�������
        double length; //���׳���
        double width; //���׿��
        if(outside_typical1 == true && outside_typical2 == true) { //��ȷ��Ϊ˫ƽ���ⲿ��׼���ף��ɼ��㷽�����
            auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //���湫���ߵı���������
            depth = sideEdge_it->second.length(); //���׵����
            auto outsideEdge_it1 = edge_curves.find(outsideAdjacentFaces1[0]->second.index); //���׹�������������������ڱ߽�
            auto outsideEdge_it2 = edge_curves.find(outsideAdjacentFaces1[2]->second.index);
            length = outsideEdge_it1->second.length(); //���׳���
            width = outsideEdge_it2->second.length(); //���׿��
            if(length < width) swap(length, width);
        }
        else if(outside_typical1 == true && outside_typical2 == false) { //��ƽ���ⲿ��׼���ף����ɼ��㷽�����
            auto outsideEdge_it1 = edge_curves.find(outsideAdjacentFaces1[0]->second.index); //���׹�������������������ڱ߽�
            auto outsideEdge_it2 = edge_curves.find(outsideAdjacentFaces1[2]->second.index);
            length = outsideEdge_it1->second.length(); //���׳���
            width = outsideEdge_it2->second.length(); //���׿��
            if(length < width) swap(length, width);
        }
        else { //�ⲿ�Ǳ�׼���ף����ɼ��㷽�����
            auto ec_it_0_2 = edge_curves.find(sideCommonEdges[0]->index); //��0�����2�湫���ߵı�����
            auto ec_it_0_3 = edge_curves.find(sideCommonEdges[1]->index); //��0�����3�湫���ߵı�����
            auto ec_it_1_2 = edge_curves.find(sideCommonEdges[2]->index); //��1�����2�湫���ߵı�����
            auto vp_it_0_2 = vertex_points.find(ec_it_0_2->second.vertex1); //��0�����2�湫�����ϵ�һ������
            auto vp_it_0_3 = vertex_points.find(ec_it_0_3->second.vertex1); //��0�����3�湫�����ϵ�һ������
            auto vp_it_1_2 = vertex_points.find(ec_it_1_2->second.vertex1); //��1�����2�湫�����ϵ�һ������
            auto cp_it_0_2 = cartesian_points.find(vp_it_0_2->second.point); //��0�����2�湫�����ϵ�һ�������
            auto cp_it_0_3 = cartesian_points.find(vp_it_0_3->second.point); //��0�����3�湫�����ϵ�һ�������
            auto cp_it_1_2 = cartesian_points.find(vp_it_1_2->second.point); //��1�����2�湫�����ϵ�һ�������
            Point p_0_2 = cp_it_0_2->second.toPoint(); //��0�����2�湫�����ϵ�һ�������
            Point p_0_3 = cp_it_0_3->second.toPoint(); //��0�����3�湫�����ϵ�һ�������
            Point p_1_2 = cp_it_1_2->second.toPoint(); //��1�����2�湫�����ϵ�һ�������
            Vector v_0_2 = sideCommonEdges[0]->getVector(); //��0�����2�湫���ߵķ�������

            Point footOfPerpendicular1 = GetFootOfPerpendicular(p_0_3, p_0_2, v_0_2); //��0�����3�湫�����ϵ�һ��������ڵ�0�����2�湫�����ϵĴ���
            Point footOfPerpendicular2 = GetFootOfPerpendicular(p_1_2, p_0_2, v_0_2); //��1�����2�湫�����ϵ�һ��������ڵ�0�����2�湫�����ϵĴ���
            length = distance_AB(footOfPerpendicular1, p_0_3); //���׳���
            width = distance_AB(footOfPerpendicular2, p_1_2); //���׿��
            if(length < width) swap(length, width);

            // //����
            // auto ec_it_1_3 = edge_curves.find(sideCommonEdges[3]->index); //��1�����3�湫���ߵı�����
            // auto vp_it_1_3 = vertex_points.find(ec_it_1_3->second.vertex1); //��1�����3�湫�����ϵ�һ������
            // auto cp_it_1_3 = cartesian_points.find(vp_it_1_3->second.point); //��1�����3�湫�����ϵ�һ�������
            // Point p_1_3 = cp_it_1_3->second.toPoint(); //��1�����3�湫�����ϵ�һ�������
            // Vector v_1_3 = sideCommonEdges[3]->getVector(); //��1�����3�湫���ߵķ�������
            // Point footOfPerpendicular3 = GetFootOfPerpendicular(p_1_2, p_1_3, v_1_3); //��1�����2�湫�����ϵ�һ��������ڵ�1�����3�湫�����ϵĴ���
            // Point footOfPerpendicular4 = GetFootOfPerpendicular(p_0_3, p_1_3, v_1_3); //��0�����3�湫�����ϵ�һ��������ڵ�1�����3�湫�����ϵĴ���
            // double length1 = distance_AB(footOfPerpendicular3, p_1_2); //���׳���
            // double width1 = distance_AB(footOfPerpendicular4, p_0_3); //���׿��
            // if(length1 < width1) swap(length1, width1);
            // cout << "length1 = " << length1 << ", width1 = " << width1 << endl;
        }

        //�ж���ϣ����
        cout << endl << "����Square hole ";
        if(inside_typical == true) cout << "�ڲ���׼Typical inside ";
        else cout << "�ڲ��Ǳ�׼Not typical inside ";
        if(outside_typical1 == true && outside_typical2 == true) cout << "˫ƽ���ⲿ��׼Double Typical outside: ";
        else if(outside_typical1 == true && outside_typical2 == false) cout << "��ƽ���ⲿ��׼Single Typical outside: ";
        else cout << "�ⲿ�Ǳ�׼Not typical outside: ";
        if(outside_typical1 == true && outside_typical2 == true) { //˫ƽ���ⲿ��׼���ף����������ȡ����ȡ����
            cout << "���depth = " << depth << ", ";
            cout << "����length = " << length << ", ���width = " << width;
        }
        else if(outside_typical1 == true && outside_typical2 == false) { //��ƽ���ⲿ��׼���ף�������ȡ����
            cout << "����length = " << length << ", ���width = " << width;
        }
        else { //�ⲿ�Ǳ�׼���ף�������ȡ����
            cout << "����length = " << length << ", ���width = " << width;
        }
        cout << " | �ĸ�����#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << endl;

        //���ιؼ���
        for(int i = 0; i < 4; i++)
        {
            faces[i]->second.setUse(1);
        }
    }
}

void hexagonalHole()
{
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        if(ad_it->second.faceType != "PLANE") continue; //����ƽ��ֱ������

        if(ad_it->second.concaveCommonLineEdge_number() < 2) continue; //������ֱ�߱߸������벻����2��

        //���ҵ�һ�Լн�120�ȵİ�����ֱ�߱ߣ���Ӧ�������������棨ƽ�棩�ֱܷ��ҵ��������������桢��һ�Լн�120�ȵİ�����ֱ�߱ߣ��̶����ҵ�һ����ͬ�İ������棨ƽ�棩����һ�Լн�120�ȵİ�����ֱ�߱ߣ�������Ҫ������ֱ�߱��໥ƽ��,ÿ����������֮�����ҽ���һ������ֱ�߱�
        vector<map<int, ADVANCED_FACE>::iterator> faces; //����������߼���ĵ���������0������1������2������3������4������5����������ڣ���β��ӣ�
        vector<CommonEdge*> sideCommonEdges; //����������֮��Ĺ�����ָ�루˳��Ϊ����0������1����֮��Ĺ����ߣ���1������2����֮��Ĺ����ߣ���2������3����֮��Ĺ����ߣ���3������4����֮��Ĺ����ߣ���4������5����֮��Ĺ����ߣ���5������0����֮��Ĺ����ߣ�
        bool found = false; //�Ƿ��ҵ�����������������
        for(auto af_it1 = ad_it->second.adjacentFaces.begin(); af_it1 != ad_it->second.adjacentFaces.end(); af_it1++)
        {
            if(advanced_faces.find(af_it1->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
            if(af_it1->second.edgeType != "LINE") continue; //ֱ�߹�����
            if(af_it1->second.concavity != 0) continue; //������ֱ�߱�
            if(!anglesAreEqual(af_it1->second.angle, (2*PI/3))) continue; //�н�Ϊ120��

            for(auto af_it2 = af_it1; af_it2 != ad_it->second.adjacentFaces.end(); af_it2++)
            {
                if(af_it1->first == af_it2->first) continue; //�����������治����ͬһ����
                if(advanced_faces.find(af_it2->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                if(af_it2->second.edgeType != "LINE") continue; //ֱ�߹�����
                if(af_it2->second.concavity != 0) continue; //������ֱ�߱�
                if(!anglesAreEqual(af_it2->second.angle, (2*PI/3))) continue; //�н�Ϊ120��
                
                if(!isParallel(af_it1->second.getVector(), af_it2->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                if(ad_it->second.commonEdge_number_withFace(af_it1->first) != 1) continue; //��������������ֱ����ҽ���һ������ֱ�߱�
                if(ad_it->second.commonEdge_number_withFace(af_it2->first) != 1) continue;

                auto afAd_it1 = advanced_faces.find(af_it1->first); //��һ����������ĸ߼��������
                auto afAd_it2 = advanced_faces.find(af_it2->first); //�ڶ�����������ĸ߼��������
                for(auto af_it3 = afAd_it1->second.adjacentFaces.begin(); af_it3 != afAd_it1->second.adjacentFaces.end(); af_it3++)
                {
                    if(af_it3->first == ad_it->first) continue; //��һ�������������һ�������治������ʼ��
                    if(advanced_faces.find(af_it3->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                    if(af_it3->second.edgeType != "LINE") continue; //ֱ�߹�����
                    if(af_it3->second.concavity != 0) continue; //������ֱ�߱�
                    if(!anglesAreEqual(af_it3->second.angle, (2*PI/3))) continue; //�н�Ϊ120��

                    if(!isParallel(af_it1->second.getVector(), af_it3->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                    if(afAd_it1->second.commonEdge_number_withFace(af_it3->first) != 1) continue; //��ǰһ�����������ҽ���һ������ֱ�߱�

                    for(auto af_it4 = afAd_it2->second.adjacentFaces.begin(); af_it4 != afAd_it2->second.adjacentFaces.end(); af_it4++)
                    {
                        if(af_it4->first == ad_it->first) continue; //�ڶ��������������һ�������治������ʼ��
                        if(advanced_faces.find(af_it4->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                        if(af_it4->second.edgeType != "LINE") continue; //ֱ�߹�����
                        if(af_it4->second.concavity != 0) continue; //������ֱ�߱�
                        if(!anglesAreEqual(af_it4->second.angle, (2*PI/3))) continue; //�н�Ϊ120��

                        if(!isParallel(af_it2->second.getVector(), af_it4->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                        if(afAd_it2->second.commonEdge_number_withFace(af_it4->first) != 1) continue; //��ǰһ�����������ҽ���һ������ֱ�߱�

                        auto afAd_it3 = advanced_faces.find(af_it3->first); //��������������ĸ߼��������
                        auto afAd_it4 = advanced_faces.find(af_it4->first); //���ĸ���������ĸ߼��������
                        for(auto af_it5 = afAd_it3->second.adjacentFaces.begin(); af_it5 != afAd_it3->second.adjacentFaces.end(); af_it5++)
                        {
                            if(af_it5->first == afAd_it1->first) continue; //����������������һ�������治���ǵ�һ����������
                            if(advanced_faces.find(af_it5->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                            if(af_it5->second.edgeType != "LINE") continue; //ֱ�߹�����
                            if(af_it5->second.concavity != 0) continue; //������ֱ�߱�
                            if(!anglesAreEqual(af_it5->second.angle, (2*PI/3))) continue; //�н�Ϊ120��

                            if(!isParallel(af_it3->second.getVector(), af_it5->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                            if(afAd_it3->second.commonEdge_number_withFace(af_it5->first) != 1) continue; //��ǰһ�����������ҽ���һ������ֱ�߱�
                            for(auto af_it6 = afAd_it4->second.adjacentFaces.begin(); af_it6 != afAd_it4->second.adjacentFaces.end(); af_it6++)
                            {
                                if(af_it5->first != af_it6->first) continue; //����͵���������������ͬһ����
                                if(af_it6->first == afAd_it2->first) continue; //�����������������һ�������治���ǵڶ�����������
                                if(advanced_faces.find(af_it6->first)->second.faceType != "PLANE") continue; //����ƽ��ֱ������
                                if(af_it6->second.edgeType != "LINE") continue; //ֱ�߹�����
                                if(af_it6->second.concavity != 0) continue; //������ֱ�߱�
                                if(!anglesAreEqual(af_it6->second.angle, (2*PI/3))) continue; //�н�Ϊ120��

                                if(!isParallel(af_it2->second.getVector(), af_it6->second.getVector())) continue; //����ֱ�߱�Ҫ�໥ƽ��

                                if(afAd_it4->second.commonEdge_number_withFace(af_it6->first) != 1) continue; //��ǰһ�����������ҽ���һ������ֱ�߱�
                                
                                //�������ǿ׵�������ĸ߼��������
                                faces.emplace_back(ad_it);
                                faces.emplace_back(afAd_it1);
                                faces.emplace_back(afAd_it3);
                                auto afAd_it5 = advanced_faces.find(af_it5->first);
                                faces.emplace_back(afAd_it5);
                                faces.emplace_back(afAd_it4);
                                faces.emplace_back(afAd_it2);

                                //�������ǿ׵�����������֮��Ĺ�����ָ��
                                sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it1->first));
                                sideCommonEdges.emplace_back(afAd_it1->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                                sideCommonEdges.emplace_back(afAd_it3->second.getCommonEdge_withAdjacentFace(afAd_it5->first));
                                sideCommonEdges.emplace_back(afAd_it5->second.getCommonEdge_withAdjacentFace(afAd_it4->first));
                                sideCommonEdges.emplace_back(afAd_it4->second.getCommonEdge_withAdjacentFace(afAd_it2->first));
                                sideCommonEdges.emplace_back(afAd_it2->second.getCommonEdge_withAdjacentFace(ad_it->first));
                                found = true; //����Ѿ��ҵ��˷���������������
                                break;
                            }
                            if(found == true) break;
                        }
                        if(found == true) break;
                    }
                    if(found == true) break;
                }
                if(found == true) break;
            }
            if(found == true) break;
        }
        if(found == false) continue; //δ�ҵ����������������棬����

        //�����治��ȫ���ѱ�ʶ��Ϊ�������棨isUsed == 1��,Ҳ�����б����ι��Ĺؼ��棨isUsed == 2��
        bool used = true;
        for(int i = 0; i < 6; i++)
        {
            if(faces[i]->second.isUsed == 2) {
                used = true;
                break;
            }
            else if(faces[i]->second.isUsed == 0) used = false;
        }
        if(used == true) continue;

        /*�ڲ���׼�ж���
        �ڲ������涼ֻ���б�Ҫ�ߣ�����һ���ջ��߽磩ʱ�ж�Ϊ�ڲ���׼������Ϊ�ڲ��Ǳ�׼��
        ����Ҫ��Ϊ����������������û�п���͹������⴦��
        */
        bool inside_typical = true; //�ڲ���׼
        for(int i = 0; i < 6; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //������ջ��߽粻��һ�����ڲ��Ǳ�׼
        }

        /*�ⲿ��׼�ж���
        ������ͬʱ��ͬһ��ƽ��������һ��270��͹ֱ�߹����ߣ���������ֱ�߱����ڲ����һ���߻���������Ҫһ���ұ߻���������
        �������������Ĺ��������������ҵ�����ʱ��Ϊ˫ƽ���ⲿ��׼���ǿף�����������ǿ���ȣ������Ĺ�������������ֻ�ҵ�һ��ʱ��Ϊ��ƽ���ⲿ��׼���ǿף�����������ǿ���ȡ�
        �����ⲿ�Ǳ�׼��
        */
        bool outside_typical1 = false, outside_typical2 = false; //�ⲿ�Ƿ��׼
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces1; //������ĵ�һ��������������
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces2; //������ĵڶ���������������
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //���ҵ���0�������������
        {
            if(af_it->second.concavity != 1) continue; //͹
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
            if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //����������ƽ��
                outsideAdjacentFaces1.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces1.size() == 2) { //���ڵ�0�����棬����������������������������֤��������ֱ�ͬ��������������������
            outsideAdjacentFaces2.emplace_back(outsideAdjacentFaces1[1]);
            outsideAdjacentFaces1.erase(outsideAdjacentFaces1.begin() + 1);
            //����֤��һ��������������
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��5�����Ƿ������������������
                {
                    if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //��i����������������0����ĵ�һ������������ͬһ����
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces1.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //��i������δ�ҵ����0����ĵ�һ������������ͬ����������
            }
            //��һ���������������������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces1.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //��һ�������������������ⲿ��׼

            //����֤�ڶ���������������
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��5�����Ƿ������������������
                {
                    if(af_it->first != outsideAdjacentFaces2[0]->first) continue; //��i����������������0����ĵڶ�������������ͬһ����
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                    if(af_it->second.concavity != 1) continue; //͹
                    if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                    outsideAdjacentFaces2.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //��i������δ�ҵ����0����ĵڶ�������������ͬ����������
            }
            //�ڶ����������������������ֱ�߱����ڲ����һ���߻�
            if(outsideAdjacentFaces2.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces2)) outside_typical2 = true; //�ڶ��������������������ⲿ��׼

            if(outside_typical2 == true && outside_typical1 == false) { //�ڶ���������������ⲿ��׼����һ���������治�����ⲿ��׼������������������
                swap(outsideAdjacentFaces1, outsideAdjacentFaces2);
                swap(outside_typical1, outside_typical2);
            }
        }
        else if(outsideAdjacentFaces1.size() > 2) { //���ڵ�0�����棬�������������������ж�����������֤��������ֱ�ͬ���������е�������������
            int num = outsideAdjacentFaces1.size(); //��0������������������
            swap(outsideAdjacentFaces1, outsideAdjacentFaces2); //����ѡ�Ĺ��������������outsideAdjacentFaces2�б���

            //�ҵ�һ��������������
            while(outside_typical1 != true && num > 0) {
                outsideAdjacentFaces1.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;
                
                for(int i = 1; i < 6; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��5�����Ƿ������������������
                    {
                        if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //��i����������������0������������������ͬһ����
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces1.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i������δ�ҵ����0������������������ͬ����������
                }
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces1.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //�������������������ⲿ��׼
                else outsideAdjacentFaces1.clear();
            }

            //�ҵ��˵�һ�������������������棬�����ҵڶ���������������������
            while(outside_typical2 != true && num > 0) {
                vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces3; //��ʱ��������洢��
                outsideAdjacentFaces3.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;

                for(int i = 1; i < 6; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //��֤ʣ��5�����Ƿ������������������
                    {
                        if(af_it->first != outsideAdjacentFaces3[0]->first) continue; //��i����������������0������������������ͬһ����
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //�н�Ϊ270��
                        if(af_it->second.concavity != 1) continue; //͹
                        if(af_it->second.edgeType != "LINE") continue; //ֱ�߹�����
                        outsideAdjacentFaces3.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //��i������δ�ҵ����0������������������ͬ����������
                }
                //����������ֱ�߱����ڲ����һ���߻�
                if(outsideAdjacentFaces3.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces3)) { //�������������������ⲿ��׼
                    outside_typical2 = true;
                    outsideAdjacentFaces2 = outsideAdjacentFaces3;
                }
            }

            //���㼸�β���
            double depth; //���ǿ����
            vector<double> length; //���ǿױ߳�

            auto ec_it_0_1 = edge_curves.find(sideCommonEdges[0]->index); //��0�����1�湫���ߵı�����
            auto ec_it_1_2 = edge_curves.find(sideCommonEdges[1]->index); //��1�����2�湫���ߵı�����
            auto ec_it_2_3 = edge_curves.find(sideCommonEdges[2]->index); //��2�����3�湫���ߵı�����
            auto ec_it_3_4 = edge_curves.find(sideCommonEdges[3]->index); //��3�����4�湫���ߵı�����
            auto ec_it_4_5 = edge_curves.find(sideCommonEdges[4]->index); //��4�����5�湫���ߵı�����
            auto ec_it_5_0 = edge_curves.find(sideCommonEdges[5]->index); //��5�����0�湫���ߵı�����
            auto vp_it_0_1 = vertex_points.find(ec_it_0_1->second.vertex1); //��0�����1�湫�����ϵ�һ������
            auto vp_it_1_2 = vertex_points.find(ec_it_1_2->second.vertex1); //��1�����2�湫�����ϵ�һ������
            auto vp_it_2_3 = vertex_points.find(ec_it_2_3->second.vertex1); //��2�����3�湫�����ϵ�һ������
            auto vp_it_3_4 = vertex_points.find(ec_it_3_4->second.vertex1); //��3�����4�湫�����ϵ�һ������
            auto vp_it_4_5 = vertex_points.find(ec_it_4_5->second.vertex1); //��4�����5�湫�����ϵ�һ������
            auto vp_it_5_0 = vertex_points.find(ec_it_5_0->second.vertex1); //��5�����0�湫�����ϵ�һ������
            auto cp_it_0_1 = cartesian_points.find(vp_it_0_1->second.point); //��0�����1�湫�����ϵ�һ�������
            auto cp_it_1_2 = cartesian_points.find(vp_it_1_2->second.point); //��1�����2�湫�����ϵ�һ�������
            auto cp_it_2_3 = cartesian_points.find(vp_it_2_3->second.point); //��2�����3�湫�����ϵ�һ�������
            auto cp_it_3_4 = cartesian_points.find(vp_it_3_4->second.point); //��3�����4�湫�����ϵ�һ�������
            auto cp_it_4_5 = cartesian_points.find(vp_it_4_5->second.point); //��4�����5�湫�����ϵ�һ�������
            auto cp_it_5_0 = cartesian_points.find(vp_it_5_0->second.point); //��5�����0�湫�����ϵ�һ�������
            Point p_0_1 = cp_it_0_1->second.toPoint(); //��0�����1�湫�����ϵ�һ�������
            Point p_1_2 = cp_it_1_2->second.toPoint(); //��1�����2�湫�����ϵ�һ�������
            Point p_2_3 = cp_it_2_3->second.toPoint(); //��2�����3�湫�����ϵ�һ�������
            Point p_3_4 = cp_it_3_4->second.toPoint(); //��3�����4�湫�����ϵ�һ�������
            Point p_4_5 = cp_it_4_5->second.toPoint(); //��4�����5�湫�����ϵ�һ�������
            Point p_5_0 = cp_it_5_0->second.toPoint(); //��5�����0�湫�����ϵ�һ�������
            Vector v_0_1 = sideCommonEdges[0]->getVector(); //��0�����1�湫���ߵķ�������
            Vector v_1_2 = sideCommonEdges[1]->getVector(); //��1�����2�湫���ߵķ�������
            Vector v_2_3 = sideCommonEdges[2]->getVector(); //��2�����3�湫���ߵķ�������
            Vector v_3_4 = sideCommonEdges[3]->getVector(); //��3�����4�湫���ߵķ�������
            Vector v_4_5 = sideCommonEdges[4]->getVector(); //��4�����5�湫���ߵķ�������
            Vector v_5_0 = sideCommonEdges[5]->getVector(); //��5�����0�湫���ߵķ�������

            Point footOfPerpendicular1 = GetFootOfPerpendicular(p_1_2, p_0_1, v_0_1); //��1�����2�湫�����ϵ�һ��������ڵ�0�����1�湫�����ϵĴ���
            Point footOfPerpendicular2 = GetFootOfPerpendicular(p_2_3, p_1_2, v_1_2); //��2�����3�湫�����ϵ�һ��������ڵ�1�����2�湫�����ϵĴ���
            Point footOfPerpendicular3 = GetFootOfPerpendicular(p_3_4, p_2_3, v_2_3); //��3�����4�湫�����ϵ�һ��������ڵ�2�����3�湫�����ϵĴ���
            Point footOfPerpendicular4 = GetFootOfPerpendicular(p_4_5, p_3_4, v_3_4); //��4�����5�湫�����ϵ�һ��������ڵ�3�����4�湫�����ϵĴ���
            Point footOfPerpendicular5 = GetFootOfPerpendicular(p_5_0, p_4_5, v_4_5); //��5�����0�湫�����ϵ�һ��������ڵ�4�����5�湫�����ϵĴ���
            Point footOfPerpendicular6 = GetFootOfPerpendicular(p_0_1, p_5_0, v_5_0); //��0�����1�湫�����ϵ�һ��������ڵ�5�����0�湫�����ϵĴ���
            length.emplace_back(distance_AB(footOfPerpendicular1, p_1_2)); //��1��ı߳�
            length.emplace_back(distance_AB(footOfPerpendicular2, p_2_3)); //��2��ı߳�
            length.emplace_back(distance_AB(footOfPerpendicular3, p_3_4)); //��3��ı߳�
            length.emplace_back(distance_AB(footOfPerpendicular4, p_4_5)); //��4��ı߳�
            length.emplace_back(distance_AB(footOfPerpendicular5, p_5_0)); //��5��ı߳�
            length.emplace_back(distance_AB(footOfPerpendicular6, p_0_1)); //��0��ı߳�
            sort(length.begin(), length.end()); //���߳���С��������
            bool isRegular = true; //�ǲ��������ǿ�
            for(int i = 1; i < 6; i++)
            {
                if(!isEqual(length[i], length[0])) isRegular = false; //�߳�����ȣ����������ǿ�
            }
            
            if(outside_typical1 == true && outside_typical2 == true) { //˫ƽ���ⲿ��׼���ǿף����Լ������ǿ����
                auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //���湫���ߵı���������
                depth = sideEdge_it->second.length(); //���ǿ����
            }

            //�ж���ϣ����
            if(isRegular == true) cout << endl << "�����ǿ�Regular Hexagonal hole ";
            else cout << endl << "���ǿ�Hexagonal hole ";
            if(inside_typical == true) cout << "�ڲ���׼Typical inside ";
            else cout << "�ڲ��Ǳ�׼Not typical inside ";
            if(outside_typical1 == true && outside_typical2 == true) cout << "˫ƽ���ⲿ��׼Double Typical outside: ";
            else if(outside_typical1 == true && outside_typical2 == false) cout << "��ƽ���ⲿ��׼Single Typical outside: ";
            else cout << "�ⲿ�Ǳ�׼Not typical outside: ";
            if(outside_typical1 == true && outside_typical2 == true) { //˫ƽ���ⲿ��׼���ǿף�������ǿ����
                cout << "���depth = " << depth << ", ";
            }
            if(isRegular == true) { //�����ǿף�������ǿױ߳�
                cout << "�߳�length = " << length[0];
            }
            else { //�������ǿף�������ǿױ߳�
                cout << "�߳�length = ";
                for(int i = 0; i < 6; i++)
                {
                    cout << length[i];
                    if(i != 5) cout << ", ";
                }
            }
            cout << " | ��������#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

            //���ιؼ���
            for(int i = 0; i < 6; i++)
            {
                faces[i]->second.setUse(1);
            }
        }
    }
}