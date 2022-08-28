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
    else if(faceType1 == "CYLINDRICAL_SURFACE" && faceType2 == "PLANE" && edgeType == "CIRCLE") {
        swap(face1, face2);
        return 3;
    }
}

void adjacency()
{
    for(auto ec_it = edge_curves.begin(); ec_it != edge_curves.end(); ec_it++)
    {
        string edgeType = ec_it->second.indexType(ec_it->second.edge); //����������
        if(edgeType != "LINE" && edgeType != "CIRCLE") continue;
        for(int up1 = 0; up1 < ec_it->second.upIndexes.size()-1; up1++) //�����øñ����ߵ����������֮����������
        {
            for(int up2 = up1+1; up2 < ec_it->second.upIndexes.size(); up2++)
            {
                auto oe_it1 = oriented_edges.find(ec_it->second.upIndexes[up1]);
                int face1 = oe_it1->second.findFace();
                auto oe_it2 = oriented_edges.find(ec_it->second.upIndexes[up2]);
                int face2 = oe_it2->second.findFace();
                int situation = adjacencySituation(face1, face2, ec_it->second.edge);
                if(situation == 1) {
                    //ƽ���ƽ�����ڣ��ཻ����ֱ��
                }
                else if(situation == 2) {
                    //ƽ����������ڣ��ཻ����ֱ��
                }
                else if(situation == 3) {
                    //ƽ����������ڣ��ཻ����Բ
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
                cout << endl << "����Բ��Circular hole: radius = " << cy_it->second.radius << endl;
            }
        }
    }
}

void cone_and_conicalHole()
{
    for(auto co_it = conical_surfaces.begin(); co_it != conical_surfaces.end(); co_it++)
    {
        auto ad_it = advanced_faces.find(co_it->second.upIndex); //�����������ҵ��߼���
        if(ad_it->second.flag == true) {
            //Բ׶Բ̨�ж�
            int v = -1; //����ѿ����������ţ�-1����û��׶���㣩
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
                    else if(fob_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��+�뾶
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
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
                    else if(fb_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��+�뾶
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
                    }
                }
            }
            //Բ׶Բ̨�ж�����
            if(v != -1 && o.size() == 1) {
                //׶�溬��һ������߽��һ��Բ�α߽�ʱ��ʶ��ΪԲ׶�����������Ϣ
                cout << endl << "Բ׶Cone: radius = " << r[0] << ", height = " << pointDistance(v, o[0]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(o.size() == 2) {
                //׶�溬������Բ�α߽�ʱ��ʶ��ΪԲ̨�����������Ϣ
                cout << endl << "Բ̨Frustum of cone: radius1 = " << max(r[0], r[1]) << ", radius2 = " << min(r[0], r[1]) << ", height = " << pointDistance(o[0], o[1]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
        }
        else {
            //Բ׶��Բ̨���ж�
            int n = 0; //�߻�ֻ����һ������ߵı߽�����
            int v = -1; //����ѿ����������ţ�-1����û��׶���㣩
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
                    else if(fob_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��+�뾶
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
                    else if(fb_it->second.isCIRCLE(circle_index) == true) { //�߽���Բ����¼Բ��+�뾶
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
            //Բ׶��Բ̨���ж�����
            if(v != -1 && o.size() == 1) {
                //׶�溬��һ������߽��һ��Բ�α߽�ʱ��ʶ��ΪԲ׶�ף����������Ϣ
                cout << endl << "Բ׶��Conical hole: radius = " << r[0] << ", height = " << pointDistance(v, o[0]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(o.size() == 2) {
                //׶�溬������Բ�α߽�ʱ��ʶ��ΪԲ̨�ף����������Ϣ
                cout << endl << "Բ̨��Frustum hole: radius1 = " << max(r[0], r[1]) << ", radius2 = " << min(r[0], r[1]) << ", height = " << pointDistance(o[0], o[1]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(v != -1) {
                //׶�溬��һ������߽�����������߽�ʱ��ʶ��Ϊ����Բ׶�ף����������Ϣ
                cout << endl << "����Բ׶��Conical hole: coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(ad_it->second.bounds.size() == 2 && n == 2) {
                //׶�����ҽ���������߽磬��ÿ����߽�ֻ��һ�������ʱ��ʶ��Ϊ����Բ׶�ף����������Ϣ
                cout << endl << "Բ̨��Frustum hole: coneAngle = " << co_it->second.coneAngle << endl;
            }
        }
    }
}