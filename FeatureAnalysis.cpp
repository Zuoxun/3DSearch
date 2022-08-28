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
    else if(faceType1 == "CYLINDRICAL_SURFACE" && faceType2 == "PLANE" && edgeType == "LINE") { //若face1是柱面face2是平面，需要交换索引号顺序
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
        string edgeType = ec_it->second.indexType(ec_it->second.edge); //边曲线类型
        if(edgeType != "LINE" && edgeType != "CIRCLE") continue;
        for(int up1 = 0; up1 < ec_it->second.upIndexes.size()-1; up1++) //对引用该边曲线的有向边两两之间找相邻面
        {
            for(int up2 = up1+1; up2 < ec_it->second.upIndexes.size(); up2++)
            {
                auto oe_it1 = oriented_edges.find(ec_it->second.upIndexes[up1]);
                int face1 = oe_it1->second.findFace();
                auto oe_it2 = oriented_edges.find(ec_it->second.upIndexes[up2]);
                int face2 = oe_it2->second.findFace();
                int situation = adjacencySituation(face1, face2, ec_it->second.edge);
                if(situation == 1) {
                    //平面和平面相邻，相交边是直线
                }
                else if(situation == 2) {
                    //平面和柱面相邻，相交边是直线
                }
                else if(situation == 3) {
                    //平面和柱面相邻，相交边是圆
                }
            }
        }
    }
}

void cylinder_and_circularHole()
{
    for(auto cy_it = cylindrical_surfaces.begin(); cy_it != cylindrical_surfaces.end(); cy_it++)
    {
        auto ad_it = advanced_faces.find(cy_it->second.upIndex); //根据上索引找到高级面
        if(ad_it->second.flag == true) {
            //圆柱判断
            vector<int> o; //圆形边界的圆心笛卡尔点索引号
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //找到边界
                    int circle_index;
                    if(fob_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //找到边界
                    int circle_index;
                    if(fb_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                }
            }
            //圆柱判定条件
            if(o.size() == 2) {
                //柱面含有两个圆形面边界时，识别为圆柱，输出特征信息
                cout << endl << "圆柱Cylinder: radius = " << cy_it->second.radius << ", height = " << pointDistance(o[0], o[1]) << endl;
            }
        }
        else {
            //圆孔判断
            int n = 0; //边环只包含一条有向边的边界数量
            vector<int> o; //圆形边界的圆心笛卡尔点索引号
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //找到边界
                    int circle_index;
                    if(fob_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心,n++
                        n++;
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                    else if(fob_it->second.isSingleEdge() == true) {  //边界不是圆，但边环只包含一条有向边
                        n++;
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //找到边界
                    int circle_index;
                    if(fb_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心,n++
                        n++;
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                    }
                    else if(fb_it->second.isSingleEdge() == true) {  //边界不是圆，但边环只包含一条有向边
                        n++;
                    }
                }
            }
            //圆孔判定条件
            if(o.size() == 2) {
                //柱面含有两个圆形面边界时，识别为平面圆孔，输出特征信息
                cout << endl << "平面圆孔Circular hole: radius = " << cy_it->second.radius << ", length = " << pointDistance(o[0], o[1]) << endl;
            }
            else if(o.size() == 1) {
                //柱面含有一个圆形面边界时，识别为圆孔，输出特征信息
                cout << endl << "圆孔Circular hole: radius = " << cy_it->second.radius << endl;
            }
            else if(ad_it->second.bounds.size() == 2 && n == 2) {
                //柱面有且仅有两个面边界，且每个面边界只含一条有向边时，识别为曲面圆孔，输出特征信息
                cout << endl << "曲面圆孔Circular hole: radius = " << cy_it->second.radius << endl;
            }
        }
    }
}

void cone_and_conicalHole()
{
    for(auto co_it = conical_surfaces.begin(); co_it != conical_surfaces.end(); co_it++)
    {
        auto ad_it = advanced_faces.find(co_it->second.upIndex); //根据上索引找到高级面
        if(ad_it->second.flag == true) {
            //圆锥圆台判断
            int v = -1; //顶点笛卡尔点索引号（-1代表没有锥顶点）
            vector<int> o; //圆形边界的圆心笛卡尔点索引号
            vector<double> r; //圆形边界的半径
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //找到边界
                    int vertex_index;
                    int circle_index;
                    if(fob_it->second.isVERTEX(vertex_index) == true) { //边界是顶点：记录顶点的笛卡尔点索引号
                        auto vp_it = vertex_points.find(vertex_index);
                        v = vp_it->second.point;
                    }
                    else if(fob_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心+半径
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //找到边界
                    int vertex_index;
                    int circle_index;
                    if(fb_it->second.isVERTEX(vertex_index) == true) { //边界是顶点：记录顶点的笛卡尔点索引号
                        auto vp_it = vertex_points.find(vertex_index);
                        v = vp_it->second.point;
                    }
                    else if(fb_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心+半径
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
                    }
                }
            }
            //圆锥圆台判定条件
            if(v != -1 && o.size() == 1) {
                //锥面含有一个顶点边界和一个圆形边界时，识别为圆锥，输出特征信息
                cout << endl << "圆锥Cone: radius = " << r[0] << ", height = " << pointDistance(v, o[0]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(o.size() == 2) {
                //锥面含有两个圆形边界时，识别为圆台，输出特征信息
                cout << endl << "圆台Frustum of cone: radius1 = " << max(r[0], r[1]) << ", radius2 = " << min(r[0], r[1]) << ", height = " << pointDistance(o[0], o[1]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
        }
        else {
            //圆锥孔圆台孔判断
            int n = 0; //边环只包含一条有向边的边界数量
            int v = -1; //顶点笛卡尔点索引号（-1代表没有锥顶点）
            vector<int> o; //圆形边界的圆心笛卡尔点索引号
            vector<double> r; //圆形边界的半径
            for(int i = 0; i < ad_it->second.bounds.size(); i++)
            {
                int bd_id = ad_it->second.bounds[i];
                if(ad_it->second.indexType(bd_id) == "FACE_OUTER_BOUND") {
                    auto fob_it = face_outer_bounds.find(bd_id); //找到边界
                    int vertex_index;
                    int circle_index;
                    if(fob_it->second.isVERTEX(vertex_index) == true) { //边界是顶点：记录顶点的笛卡尔点索引号
                        auto vp_it = vertex_points.find(vertex_index);
                        v = vp_it->second.point;
                    }
                    else if(fob_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心+半径
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
                    }
                    else if(fob_it->second.isSingleEdge() == true) { //边界不是顶点也不是圆，但边环只包含一条有向边
                        n++;
                    }
                }
                if(ad_it->second.indexType(bd_id) == "FACE_BOUND") {
                    auto fb_it = face_bounds.find(bd_id); //找到边界
                    int vertex_index;
                    int circle_index;
                    if(fb_it->second.isVERTEX(vertex_index) == true) { //边界是顶点：记录顶点的笛卡尔点索引号
                        auto vp_it = vertex_points.find(vertex_index);
                        v = vp_it->second.point;
                    }
                    else if(fb_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆心+半径
                        auto cc_it = circles.find(circle_index);
                        auto ax_it = axis2_pacement_3ds.find(cc_it->second.axis2);
                        o.emplace_back(ax_it->second.point);
                        r.emplace_back(cc_it->second.radius);
                    }
                    else if(fb_it->second.isSingleEdge() == true) { //边界不是顶点也不是圆，但边环只包含一条有向边
                        n++;
                    }
                }
            }
            //圆锥孔圆台孔判定条件
            if(v != -1 && o.size() == 1) {
                //锥面含有一个顶点边界和一个圆形边界时，识别为圆锥孔，输出特征信息
                cout << endl << "圆锥孔Conical hole: radius = " << r[0] << ", height = " << pointDistance(v, o[0]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(o.size() == 2) {
                //锥面含有两个圆形边界时，识别为圆台孔，输出特征信息
                cout << endl << "圆台孔Frustum hole: radius1 = " << max(r[0], r[1]) << ", radius2 = " << min(r[0], r[1]) << ", height = " << pointDistance(o[0], o[1]) << ", coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(v != -1) {
                //锥面含有一个顶点边界和任意其他边界时，识别为曲面圆锥孔，输出特征信息
                cout << endl << "曲面圆锥孔Conical hole: coneAngle = " << co_it->second.coneAngle << endl;
            }
            else if(ad_it->second.bounds.size() == 2 && n == 2) {
                //锥面有且仅有两个面边界，且每个面边界只含一条有向边时，识别为曲面圆锥孔，输出特征信息
                cout << endl << "圆台孔Frustum hole: coneAngle = " << co_it->second.coneAngle << endl;
            }
        }
    }
}