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
    else if(faceType1 == "CYLINDRICAL_SURFACE" && faceType2 == "PLANE" && edgeType == "CIRCLE") { //若face1是柱面face2是平面，需要交换索引号顺序
        swap(face1, face2);
        return 3;
    }
    else return 0; //非以上情况返回0
}

void makeAAG()
{
    for(auto ec_it = edge_curves.begin(); ec_it != edge_curves.end(); ec_it++)
    {
        string edgeType = ec_it->second.indexType(ec_it->second.edge); //边曲线类型
        if(edgeType != "LINE" && edgeType != "CIRCLE") continue; //仅寻找公共边是直线或圆的相邻面
        for(int up1 = 0; up1 < ec_it->second.upIndexes.size()-1; up1++) //对引用该边曲线的有向边两两之间找相邻面
        {
            for(int up2 = up1+1; up2 < ec_it->second.upIndexes.size(); up2++)
            {
                auto oe_it1 = oriented_edges.find(ec_it->second.upIndexes[up1]); //有向边1
                int face1 = oe_it1->second.findFace(); //相邻面1
                auto oe_it2 = oriented_edges.find(ec_it->second.upIndexes[up2]); //有向边2
                int face2 = oe_it2->second.findFace(); //相邻面2
                int situation = adjacencySituation(face1, face2, ec_it->second.index); //相邻情况(face1自动转为基面)
                if(situation == 1) {
                    //平面和平面相邻，公共边是直线

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it1 = planes.find(ad_it1->second.face);
                    auto pl_it2 = planes.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it1->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(pl_it2->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //平面1的Z轴向量
                    Vector dirZ2 = dirZ_it2->second.getVector(); //平面2的Z轴向量
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //平面1的外法向量N1
                    Vector N2 = (ad_it2->second.flag == true) ? dirZ2 : toOppositeVector(dirZ2); //平面2的外法向量N2

                    auto line_it = lines.find(ec_it->second.edge);
                    auto vec_it = vectors.find(line_it->second.vector);
                    auto dirE_it = directions.find(vec_it->second.direction);
                    Vector dirE = dirE_it->second.getVector();
                    Vector Ne = (oe_it1->second.flag == ec_it->second.flag) ? dirE : toOppositeVector(dirE); //以平面1为基面时的边向量Ne

                    Vector N = Ne ^ N2; //向量叉乘，N的物理意义是平面F2内垂直于公共线e向平面内部的方向向量
                    
                    //cout << "有向边1：#" << oe_it1->first << ",有向边2：#" << oe_it2->first << ",边曲线：#" << ec_it->first << "。Ne: " << Ne.show() << ", N1: " << N1.show() << ", N2: " << N2.show() << ", N: " << N.show() << endl;
                    
                    double theta = getAngle(N, N1); // N 与 N1 的夹角θ
                    
                    //cout << "θ = " << theta << endl;

                    int concavity; //凹凸性
                    if(fabs(theta) > (PI / 2.0)) concavity = 1; // |θ| > π/2时为凸
                    else if(fabs(theta) < (PI / 2.0)) concavity = 0; // |θ| < π/2时为凹
                    else concavity = -1; // |θ| = π/2时为非凹非凸

                    double angleN = getAbsAngle(N1, N2); //两平面外法向量的夹角绝对值
                    double angle;//两平面的外表面夹角
                    if(concavity == 1) {
                        angle = angleN + PI;
                    }
                    else if(concavity == 0) {
                        angle = PI - angleN;
                    }
                    else angle = PI;

                    CommonEdge commonEdge(ec_it->second.index, "LINE", concavity, angle); //公共直线边
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //两平面的相邻关系
                    ad_it2->second.adjacentFaces.emplace(ad_it1->second.index, commonEdge);
                }
                else if(situation == 2) {
                    //平面和柱面相邻，公共边是直线

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it = planes.find(ad_it1->second.face);
                    auto cy_it = cylindrical_surfaces.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(cy_it->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //平面的Z轴向量
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //平面的外法向量N1

                    auto vp_it = vertex_points.find(ec_it->second.vertex1);
                    auto cp_it1 = cartesian_points.find(vp_it->second.point);
                    Point pt = cp_it1->second.toPoint(); //公共直线边上一点P
                    auto cp_it2 = cartesian_points.find(ax_it2->second.point);
                    Point begin = cp_it2->second.toPoint(); //柱面轴心射线起点
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ2 = dirZ_it2->second.getVector(); //柱面的Z轴向量（轴心射线向量）
                    Vector perpendicularVec = GetPerpendicular(pt, begin, dirZ2); //公共边上一点向柱轴心射线的垂线向量
                    Vector N2 = (ad_it2->second.flag == false) ? perpendicularVec : toOppositeVector(perpendicularVec); //柱面在公共直线边上一点P的外法向量N2

                    auto line_it = lines.find(ec_it->second.edge);
                    auto vec_it = vectors.find(line_it->second.vector);
                    auto dirE_it = directions.find(vec_it->second.direction);
                    Vector dirE = dirE_it->second.getVector();
                    auto pl_oe_it = (oe_it1->second.findFace() == face1) ? oe_it1 : oe_it2;
                    Vector Ne = (pl_oe_it->second.flag == ec_it->second.flag) ? dirE : toOppositeVector(dirE); //以平面为基面时的边向量Ne
                    
                    Vector N = Ne ^ N2; //向量叉乘，N的物理意义是柱面在与平面的公共直线边上的一个点上，垂直于公共边向柱面方向的切线向量

                    double theta = getAngle(N, N1); // N 与 N1 的夹角θ
                    int concavity; //凹凸性
                    if(fabs(theta) > PI / 2) concavity = 1; // |θ| > π/2时为凸
                    else if(fabs(theta) < PI / 2) concavity = 0; // |θ| < π/2时为凹
                    else concavity = -1; // |θ| = π/2时为非凹非凸

                    double angleN = getAbsAngle(N1, N2); //平面和柱面外法向量的夹角绝对值
                    double angle;//平面和柱面的外表面夹角
                    if(concavity == 1) {
                        angle = angleN + PI;
                    }
                    else if(concavity == 0) {
                        angle = PI - angleN;
                    }
                    else angle = PI;

                    CommonEdge commonEdge(ec_it->second.index, "LINE", concavity, angle); //公共直线边
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //平面和柱面的相邻关系
                    ad_it2->second.adjacentFaces.emplace(ad_it1->second.index, commonEdge);
                }
                else if(situation == 3) {
                    //平面和柱面相邻，公共边是圆

                    auto ad_it1 = advanced_faces.find(face1);
                    auto ad_it2 = advanced_faces.find(face2);
                    auto pl_it = planes.find(ad_it1->second.face);
                    auto cy_it = cylindrical_surfaces.find(ad_it2->second.face);
                    auto ax_it1 = axis2_pacement_3ds.find(pl_it->second.axis2);
                    auto ax_it2 = axis2_pacement_3ds.find(cy_it->second.axis2);
                    auto dirZ_it1 = directions.find(ax_it1->second.directionZ);
                    Vector dirZ1 = dirZ_it1->second.getVector(); //平面的Z轴向量
                    Vector N1 = (ad_it1->second.flag == true) ? dirZ1 : toOppositeVector(dirZ1); //平面的外法向量N1

                    auto vp_it = vertex_points.find(ec_it->second.vertex1);
                    auto cp_it1 = cartesian_points.find(vp_it->second.point);
                    Point pt = cp_it1->second.toPoint(); //公共圆边上一点P
                    auto cp_it2 = cartesian_points.find(ax_it2->second.point);
                    Point begin = cp_it2->second.toPoint(); //柱面轴心射线起点
                    auto dirZ_it2 = directions.find(ax_it2->second.directionZ);
                    Vector dirZ2 = dirZ_it2->second.getVector(); //柱面的Z轴向量（轴心射线向量）
                    Vector perpendicularVec = GetPerpendicular(pt, begin, dirZ2); //公共边上一点向柱轴心射线的垂线向量
                    Vector PVec = perpendicularVec; //PVec

                    Vector N2 = (ad_it2->second.flag == false) ? perpendicularVec : toOppositeVector(perpendicularVec); //柱面在公共圆边上一点P的外法向量N2
                    
                    auto cc_it = circles.find(ec_it->second.edge);
                    auto ax_it3 = axis2_pacement_3ds.find(cc_it->second.axis2);
                    auto dirZ_it3 = directions.find(ax_it3->second.directionZ);
                    Vector dirZ3 = dirZ_it3->second.getVector(); // 圆边曲线的Z轴向量
                    Vector Vec = dirZ3; //Vec

                    auto pl_oe_it = (oe_it1->second.findFace() == face1) ? oe_it1 : oe_it2;
                    Vector Ne = (pl_oe_it->second.flag == ec_it->second.flag) ? (PVec ^ Vec) : (Vec ^ PVec); //以平面为基面时公共圆边上一点P的切线边向量Ne

                    Vector N = Ne ^ N2; //向量叉乘，N的物理意义是柱面在与平面的公共圆形边上的一个点上，垂直于公共边向柱面方向的切线向量

                    double theta = getAngle(N, N1); // N 与 N1 的夹角θ
                    int concavity; //凹凸性
                    if(fabs(theta) > PI / 2) concavity = 1; // |θ| > π/2时为凸
                    else if(fabs(theta) < PI / 2) concavity = 0; // |θ| < π/2时为凹

                    double angle;//平面和柱面的外表面夹角
                    if(concavity == 1) {
                        angle = PI * 1.5;
                    }
                    else if(concavity == 0) {
                        angle = PI * 0.5;
                    }

                    CommonEdge commonEdge(ec_it->second.index, "CIRCLE", concavity, angle); //公共圆形边
                    ad_it1->second.adjacentFaces.emplace(ad_it2->second.index, commonEdge); //平面和柱面的相邻关系
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
                cout << endl << "曲面圆孔Curved circular hole: radius = " << cy_it->second.radius << endl;
            }
        }
    }
}

void cone_and_conicalHole()
{
    for(auto co_it = conical_surfaces.begin(); co_it != conical_surfaces.end(); co_it++)
    {
        auto ad_it = advanced_faces.find(co_it->second.upIndex); //根据上索引找到高级面
        
        //圆锥圆台判定条件准备
        int n = 0; //边环只包含一条有向边的边界数量
        int v = -1; //顶点笛卡尔点索引号（-1代表没有锥顶点）
        vector<int> c; //圆形边界的CIRCLE索引号
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
                else if(fob_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆+圆心+半径
                    c.emplace_back(circle_index);
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
                else if(fb_it->second.isCIRCLE(circle_index) == true) { //边界是圆：记录圆+圆心+半径
                    c.emplace_back(circle_index);
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

        //特征判定
        if(vertex_loops.size() > 0) {
            //有顶点标准
            if(ad_it->second.flag == true) {
                //圆锥圆台相关
                if(v != -1 && c.size() == 1) {
                    //圆锥：锥面+T+有一个VERTEX_LOOP边界+一个CIRCLE边界
                    cout << endl << "圆锥Cone: 底面半径bottom radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v != -1 && c.size() == 0) {
                    //曲面圆锥：锥面+T+有一个VERTEX_LOOP边界+没有CIRCLE边界
                    cout << endl << "曲面圆锥Curved cone: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 2) {
                    //圆台：锥面+T+有两个CIRCLE边界+没有VERTEX_LOOP边界
                    cout << endl << "圆台Frustum: 大半径radius1 = " << max(r[0], r[1]) << ", 小半径radius2 = " << min(r[0], r[1]) << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //单曲面圆台：锥面+T+有一个CIRCLE边界+一个及以上单边界+没有VERTEX_LOOP边界
                    cout << endl << "单曲面圆台Single curved frustum: 半径radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 0 && n >= 2) {
                    //双曲面圆台：锥面+T+有两个及以上单边界+没有CIRCLE边界+没有VERTEX_LOOP边界（不标准、允许bug存在）
                    cout << endl << "双曲面圆台Double curved frustum: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
            else {
                //圆锥孔圆台孔相关
                if(v != -1 && c.size() == 1) {
                    //圆锥孔：锥面+F+有一个VERTEX_LOOP边界+一个CIRCLE边界
                    cout << endl << "圆锥孔Conical hole: 孔径hole radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v != -1 && c.size() == 0) {
                    //曲面圆锥孔：锥面+F+有一个VERTEX_LOOP边界+没有CIRCLE边界
                    cout << endl << "曲面圆锥孔Curved conical hole: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 2) {
                    //圆台孔：锥面+F+有两个CIRCLE边界+没有VERTEX_LOOP边界
                    cout << endl << "圆台孔Frustum hole: 大半径radius1 = " << max(r[0], r[1]) << ", 小半径radius2 = " << min(r[0], r[1]) << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //平面曲底圆台孔：锥面+F+有且仅有一个CIRCLE边界（凸）+一个及以上单边界+没有VERTEX_LOOP边界
                    cout << endl << "平面曲底圆台孔Frustum hole with flat surface and curved bottom: 孔径hole radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 1 && n >= 1) {
                    //曲面平底圆台孔：锥面+F+有且仅有一个CIRCLE边界（凹）+一个及以上单边界+没有VERTEX_LOOP边界
                    cout << endl << "曲面平底圆台孔Frustum hole with curved surface and flat bottom: 底面半径bottom radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(v == -1 && c.size() == 0 && n >= 2) {
                    //曲面曲底圆台孔：锥面+F+有两个及以上单边界+没有CIRCLE边界+没有VERTEX_LOOP边界（不标准、允许bug存在）
                    cout << endl << "曲面曲底圆台孔Frustum hole with curved surface and curved bottom: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
        }
        else {
            //无顶点标准
            if(ad_it->second.flag == true) {
                //圆锥圆台相关
                if(ad_it->second.bounds.size() == 1 && c.size() == 1) {
                    //圆锥：锥面+T+有且仅有一个边界，边界是CIRCLE（标准）
                    cout << endl << "圆锥Cone: 底面半径bottom radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n == 1 && c.size() == 0) {
                    //曲面圆锥：锥面+T+有且仅有一个单边界+没有CIRCLE边界（不标准、允许bug存在）
                    cout << endl << "曲面圆锥Curved cone: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n >= 2 && c.size() == 0) {
                    //曲面圆锥/双曲面圆台：锥面+T+有两个及以上单边界+没有CIRCLE边界（不标准、允许bug存在，分类模糊）
                    cout << endl << "曲面圆锥Curved cone/双曲面圆台Double curved frustum: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 2) {
                    //圆台：锥面+T+有两个CIRCLE边界（标准）
                    cout << endl << "圆台Frustum: 大半径radius1 = " << max(r[0], r[1]) << ", 小半径radius2 = " << min(r[0], r[1]) << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //圆锥/单曲面圆台：锥面+T+有且仅有一个CIRCLE边界+一个及以上单边界（分类模糊）
                    cout << endl << "圆锥Cone/单曲面圆台Single curved frustum: 半径radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
            }
            else {
                //圆锥孔圆台孔相关
                if(ad_it->second.bounds.size() == 1 && c.size() == 1) {
                    //圆锥孔：锥面+F+有且仅有一个边界，边界是CIRCLE（标准）
                    cout << endl << "圆锥孔Conical hole: 孔径hole radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n == 1 && c.size() == 0) {
                    //曲面圆锥孔：锥面+F+有且仅有一个单边界+没有CIRCLE边界（不标准、允许bug存在）
                    cout << endl << "曲面圆锥孔Curved conical hole: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(n >= 2 && c.size() == 0) {
                    //曲面圆锥孔/曲面曲底圆台孔：锥面+F+有两个及以上单边界+没有CIRCLE边界（不标准、允许bug存在，分类模糊）
                    cout << endl << "曲面圆锥孔Curved conical hole/曲面曲底圆台孔Frustum hole with curved surface and curved bottom: 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 2) {
                    //圆台孔：锥面+F+有两个CIRCLE边界（标准）
                    cout << endl << "圆台孔Frustum hole: 大半径radius1 = " << max(r[0], r[1]) << ", 小半径radius2 = " << min(r[0], r[1]) << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //圆锥孔/平面曲底圆台孔：锥面+F+有且仅有一个CIRCLE边界（凸）+一个及以上单边界（分类模糊）
                    cout << endl << "圆锥孔Conical hole/平面曲底圆台孔Frustum hole with flat surface and curved bottom: 孔径hole radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
                }
                else if(c.size() == 1 && n >= 1) {
                    //曲面平底圆台孔：锥面+F+有且仅有一个CIRCLE边界（凹）+一个及以上单边界（不标准、允许bug存在，分类模糊）
                    cout << endl << "曲面平底圆台孔Frustum hole with curved surface and flat bottom: 底面半径bottom radius = " << r[0] << ", 顶锥角coneAngle = " << co_it->second.coneAngle << endl;
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
        if(ad_it->second.faceType != "PLANE") continue; //不是平面直接跳过

        if(ad_it->second.concaveCommonLineEdge_number() < 4) continue; //凹公共直线边个数必须不少于4个
        //cout << "Plane" << ad_it->first<<endl;

        //在底面中能找到一个尺寸最大的由4个凹公共直线边组成的闭环边界（比较边长）（为了规避底面内部可能出现的四棱柱）
        vector<map<int, CommonEdge>::iterator> concaveAdjacentFaces; //最大闭环边界凹相邻面的迭代器
        if(ad_it->second.get_4concaveAdjacentFaces_closedLoopOutside(concaveAdjacentFaces) == false) continue;
        //cout << "curve " << concaveAdjacentFaces[0]->second.index << ' ' << concaveAdjacentFaces[1]->second.index << ' ' << concaveAdjacentFaces[2]->second.index << ' ' << concaveAdjacentFaces[3]->second.index << ' ' << endl;

        //4个凹相邻面必须都是平面
        vector<map<int, ADVANCED_FACE>::iterator> faces; //相邻面高级面的迭代器
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[0]->first));
        if(faces[0]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[1]->first));
        if(faces[1]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[2]->first));
        if(faces[2]->second.faceType != "PLANE") continue;
        faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[3]->first));
        if(faces[3]->second.faceType != "PLANE") continue;
        // map<int, CommonEdge>::iterator it = ad_it->second.adjacentFaces.begin(); //map<相邻面索引号, 公共边曲线>迭代器
        // int index1 = it->first; //4个相邻面的索引号
        // map<int, PLANE>::iterator plane1 = planes.find(index1); //4个相邻面
        // if(plane1 == planes.end()) continue; //4个相邻面必须都是平面
        // map<int, CommonEdge>::iterator commonEdge1 = it; //每个相邻面对应的公共边
        // it++;
        // int index2 = it->first;
        // map<int, PLANE>::iterator plane2 = planes.find(index2);
        // if(plane2 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge2 = it;
        // it++;
        // int index3 = it->first;
        // map<int, PLANE>::iterator plane3 = planes.find(index3);
        // if(plane3 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge3 = it; //每个相邻面对应的公共边
        // it++;
        // int index4 = it->first;
        // map<int, PLANE>::iterator plane4 = planes.find(index4);
        // if(plane4 == planes.end()) continue;
        // map<int, CommonEdge>::iterator commonEdge4 = it; //每个相邻面对应的公共边
        // it++;

        //与凹相邻面的凹边边夹角必须为π/2（90度）
        if(!anglesAreEqual(concaveAdjacentFaces[0]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[1]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[2]->second.angle, (PI/2))) continue;
        if(!anglesAreEqual(concaveAdjacentFaces[3]->second.angle, (PI/2))) continue;

        //四个侧相邻面之间有且仅有四个公共直线边，且每两个侧相邻面之间有一个，并求出这四个公共直线边
        vector<CommonEdge*> sideCommonEdges; //四个侧相邻面两两之间的公共边指针
        if(reorderAdjacentFaces(concaveAdjacentFaces, faces, sideCommonEdges) == false) continue; //重排序四个侧相邻面

        //这四个公共边都是凹边且夹角为π/2（90度）
        if(sideCommonEdges[0]->concavity != 0 || !anglesAreEqual(sideCommonEdges[0]->angle, (PI/2))) continue;
        if(sideCommonEdges[1]->concavity != 0 || !anglesAreEqual(sideCommonEdges[1]->angle, (PI/2))) continue;
        if(sideCommonEdges[2]->concavity != 0 || !anglesAreEqual(sideCommonEdges[2]->angle, (PI/2))) continue;
        if(sideCommonEdges[3]->concavity != 0 || !anglesAreEqual(sideCommonEdges[3]->angle, (PI/2))) continue;

        //这四个公共边与底面垂直，且互相之间平行
        vector<Vector> sideVec; //四个公共直线边的向量
        for(int i = 0; i < 4; i++)
        {
            sideVec.emplace_back(sideCommonEdges[i]->getVector());
        }
        Vector bottomFaceVec = ad_it->second.getDirectionZ(); //底面Z轴向量
        if(isParallel(sideVec[0], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[1], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[2], bottomFaceVec) == false) continue;
        if(isParallel(sideVec[3], bottomFaceVec) == false) continue;
        if(allParallel(sideVec) == false) continue;
        //cout << "All Parallel" << endl;

        /*内部标准判定：
        内部五个面都只含有必要边（仅有一个闭环边界）时判定为内部标准，否则为内部非标准
        （主要是为了区分五个面内有没有开孔凸起等特殊处理）
        */
        bool inside_typical = true; //内部是否标准
        if(ad_it->second.bounds.size() != 1) inside_typical = false; //底面闭环边界不仅一个，内部非标准
        //cout << "inside bounds.size = " << ad_it->second.bounds.size() << endl;
        for(int i = 0; i < 4; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //侧面闭环边界不仅一个，内部非标准
            //cout << "outside[" << i << "].bounds.size = " << faces[i]->second.bounds.size() << endl;
        }

        /*外部标准判定：
        四个侧面同时与同一个平面（除了底面）相邻于一个270度凸直线公共边，且这四条直线边属于并组成一个边环（这里需要一个找边环函数），（且四条边相邻垂直）。——外部标准，可以输出腔深度
        否则外部非标准。
        */
        bool outside_typical = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //四个侧面的外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个侧面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
            if(af_it->second.edgeType != "LINE") continue; //直线公共边
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //外相邻面是平面
                outsideAdjacentFaces.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces.size() == 1) { //对于第0个侧面，符合条件的外相邻面有且仅有一个，则继续判断
            //cout << "outsideFace = " << outsideAdjacentFaces[0]->first << ", outsideEdge[0] = " << outsideAdjacentFaces[0]->second.index << endl;
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //再找剩下3个侧面的外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces[0]->first) continue; //第i个侧面的外相邻面与第0个侧面的外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces.emplace_back(af_it);
                    found = true;
                    //cout << "outsideEdge[" << i << "] = " << af_it->second.index << endl;
                    break;
                }
                if(found == false) break; //第i个侧面中未找到与第0个侧面外相邻面相同的外相邻面
            }
            //四条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //条件符合外部标准
        }
        else if(outsideAdjacentFaces.size() > 1) { // 第0个侧面找到符合条件的外相邻面多于1个，要与对位的第1个侧面比对共同的外相邻面是哪个,再继续判断
            bool foundCommonFace = false; // 是否找到第0侧面与第1侧面的公共外相邻面
            for(auto af_it = faces[1]->second.adjacentFaces.begin(); af_it != faces[1]->second.adjacentFaces.end(); af_it++) //找到与第0个侧面对位的第1个侧面的外相邻面，对比得到公共外相邻面
            {
                if(af_it->second.concavity != 1) continue; //凸
                if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                if(af_it->second.edgeType != "LINE") continue; //直线公共边
                for(auto oaf_itt = outsideAdjacentFaces.begin(); oaf_itt != outsideAdjacentFaces.end(); oaf_itt++)
                {
                    if(af_it->first == (*oaf_itt)->first) { //找到公共外相邻面
                        outsideAdjacentFaces.clear();
                        outsideAdjacentFaces.emplace_back(*oaf_itt);
                        outsideAdjacentFaces.emplace_back(af_it);
                        foundCommonFace = true;
                        break;
                    }
                }
                if(foundCommonFace == true) break; //找到了公共外相邻面
            }
            //与第2侧面、第3侧面继续判断公共外相邻面
            if(foundCommonFace == true) {
                for(int i = 2; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //再找剩下2个侧面的外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces[0]->first) continue; //第i个侧面的外相邻面与第0个侧面的外相邻面是同一个面
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个侧面中未找到与第0个侧面外相邻面相同的外相邻面
                }
                //四条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //条件符合外部标准
            }
        }
        
        //若确定为外部标准型腔，计算几何参数
        auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //侧相邻面公共边的边曲线索引
        double depth = sideEdge_it->second.length(); //型腔的深度
        auto bottomEdge_it1 = edge_curves.find(concaveAdjacentFaces[0]->second.index); //底面的两个相邻边界
        auto bottomEdge_it2 = edge_curves.find(concaveAdjacentFaces[2]->second.index);
        double length = bottomEdge_it1->second.length(); //底面长度
        double width = bottomEdge_it2->second.length(); //底面宽度
        if(length < width) swap(length, width);

        //判定完毕，输出
        cout << endl << "型腔Closed pocket ";
        if(inside_typical == true) cout << "内部标准Typical inside ";
        else cout << "内部非标准Not typical inside ";
        if(outside_typical == true) cout << "外部标准Typical outside: ";
        else cout << "外部非标准Not typical outside: ";
        if(outside_typical == true) cout << "深度depth = " << depth << ", ";
        cout << "底面长度length = " << length << ", 底面宽度width = " << width;
        cout << " | 底面#" << ad_it->first << ", 四个侧面#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << endl;

        //屏蔽关键面
        ad_it->second.setUse(2); //底面为关键面
        faces[0]->second.setUse(1); //侧面被识别为特征
        faces[1]->second.setUse(1);
        faces[2]->second.setUse(1);
        faces[3]->second.setUse(1);
    }
}
bool reorderAdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges)
{
    //先粗略调整凹相邻面的顺序，把与第0个面没有公共边的面提到第1位
    if(faces[0]->second.commonEdge_number_withFace(faces[2]->first) == 0) {
        swap(concaveAdjacentFaces[1], concaveAdjacentFaces[2]);
        swap(faces[1], faces[2]);
    }
    else if(faces[0]->second.commonEdge_number_withFace(faces[3]->first) == 0) {
        swap(concaveAdjacentFaces[1], concaveAdjacentFaces[3]);
        swap(faces[1], faces[3]);
    }
    //确保不相邻的对位面之间没有公共边（第0个面、第1个面之间，第2个面、第3个面之间）
    if(faces[0]->second.commonEdge_number_withFace(faces[1]->first) != 0) return false;
    if(faces[2]->second.commonEdge_number_withFace(faces[3]->first) != 0) return false;

    //两个侧相邻面之间有且仅有一个公共直线边
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


    //验证了四个侧相邻面的相邻关系，完成了面的重新排序和公共直线边的获取后，返回成功
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
    bool isLoop = false; //一组公共边是否属于并组成一个边环
    for(auto el_it = edge_loops.begin(); el_it != edge_loops.end(); el_it++)
    {
        if(el_it->second.edges.size() != outsideAdjacentFaces.size()) continue; //该边环含有边个数与公共边个数相同
        bool belong = false; //遍历边环中的每条边是否是这组公共边之一
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
        if(ad_it->second.faceType != "PLANE") continue; //不是平面直接跳过

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
        if(ad_it->second.concaveCommonLineEdge_number() < 6) continue; //凹公共直线边个数必须不少于6个
        //cout << "face" << ad_it->first << endl;

        //在底面中能找到一个尺寸最大的由6个同样长度的凹公共直线边组成的闭环边界（比较边长）（为了规避底面内部可能出现的六棱柱）
        vector<map<int, CommonEdge>::iterator> concaveAdjacentFaces; //最大闭环边界凹相邻面的迭代器
        if(ad_it->second.get_6concaveAdjacentFaces_closedLoopOutside(concaveAdjacentFaces) == false) continue;

        //6个凹相邻面必须都是平面
        bool allPlane = true;
        vector<map<int, ADVANCED_FACE>::iterator> faces; //相邻面高级面的迭代器
        for(int i = 0; i < concaveAdjacentFaces.size(); i++)
        {
            faces.emplace_back(advanced_faces.find(concaveAdjacentFaces[i]->first));
            if(faces[i]->second.faceType != "PLANE") {
                allPlane = false;
                break;
            }
        }
        if(allPlane == false) continue;

        //与凹相邻面的凹边边夹角必须为π/2（90度）
        bool allVertical = true;
        for(int i = 0; i < concaveAdjacentFaces.size(); i++)
        {
            if(!anglesAreEqual(concaveAdjacentFaces[i]->second.angle, (PI/2))) {
                allVertical = false;
                break;
            }
        }
        if(allVertical == false) continue;

        //六个侧相邻面之间有且仅有六个公共直线边，且每两个侧相邻面之间有一个，并求出这六个公共直线边
        vector<CommonEdge*> sideCommonEdges; //六个侧相邻面两两之间的公共边指针
        if(reorder6AdjacentFaces(concaveAdjacentFaces, faces, sideCommonEdges) == false) continue; //重排序六个侧相邻面

        //这六个公共边都是凹边且夹角为2π/3（120度）。
        bool allConcaveAngle120 = true;
        for(int i = 0; i < 6; i++)
        {
            if(sideCommonEdges[i]->concavity != 0 || !anglesAreEqual(sideCommonEdges[i]->angle, (2*PI/3))) {
                allConcaveAngle120 = false;
                break;
            }
        }
        if(allConcaveAngle120 == false) continue;

        //这六个公共边与底面垂直，且互相之间平行。
        bool areAllParallel = true;
        Vector bottomFaceVec = ad_it->second.getDirectionZ(); //底面Z轴向量
        vector<Vector> sideVec; //六个公共直线边的向量
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

        /*内部标准判定：
        内部七个面都只含有必要边（仅有一个闭环边界）时判定为内部标准，否则为内部非标准
        （主要是为了区分七个面内有没有开孔凸起等特殊处理）
        */
        bool inside_typical = true; //内部是否标准
        if(ad_it->second.bounds.size() != 1) inside_typical = false; //底面闭环边界不仅一个，内部非标准
        for(int i = 0; i < 6; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //侧面闭环边界不仅一个，内部非标准
        }

        /*外部标准判定：
        六个侧面同时与同一个平面（除了底面）相邻于一个270度凸直线公共边，且这六条直线边属于并组成一个边环（这里需要一个找边环函数），（且六条边相邻垂直）。——外部标准，可以输出腔深度
        否则外部非标准。
        */
        bool outside_typical = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //六个侧面的外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个侧面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
            if(af_it->second.edgeType != "LINE") continue; //直线公共边
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //外相邻面是平面
                outsideAdjacentFaces.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces.size() == 1) { //对于第0个侧面，符合条件的外相邻面有且仅有一个，则继续判断
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //再找剩下5个侧面的外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces[0]->first) continue; //第i个侧面的外相邻面与第0个侧面的外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //第i个侧面中未找到与第0个侧面外相邻面相同的外相邻面
            }
            //六条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //条件符合外部标准

            // //若确定为外部标准六角型腔，计算几何参数
            // auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //侧相邻面公共边的边曲线索引
            // double depth = sideEdge_it->second.length(); //六角型腔的深度
            // auto bottomEdge_it = edge_curves.find(concaveAdjacentFaces[0]->second.index); //底面的一个相邻边界
            // double length = bottomEdge_it->second.length(); //底面边长

            // //判定完毕，输出
            // cout << endl << "六角型腔Hexagonal Closed pocket ";
            // if(inside_typical == true) cout << "内部标准Typical inside ";
            // else cout << "内部非标准Not typical inside ";
            // if(outside_typical == true) cout << "外部标准Typical outside: ";
            // else cout << "外部非标准Not typical outside: ";
            // if(outside_typical == true) cout << "深度depth = " << depth << ", ";
            // cout << "底面边长length = " << length;
            // cout << " | 底面#" << ad_it->first << ", 六个侧面#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

            // //屏蔽关键面
            // ad_it->second.setUse(2);
            // for(int i = 0; i < 6; i++)
            // {
            //     faces[i]->second.setUse(1);
            // }
        }
        else if(outsideAdjacentFaces.size() > 1) { // 第0个侧面找到符合条件的外相邻面多于1个，要与不相邻的第2个侧面比对共同的外相邻面是哪个,再继续判断
            bool foundCommonFace = false; //是否找到不相邻的第2个侧面与第0个侧面有共同的外相邻面
            for(auto af_it = faces[2]->second.adjacentFaces.begin(); af_it != faces[2]->second.adjacentFaces.end(); af_it++) //找到与第0个侧面不相邻的第2个侧面的外相邻面，对比得到公共外相邻面
            {
                if(af_it->second.concavity != 1) continue; //凸
                if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                if(af_it->second.edgeType != "LINE") continue; //直线公共边
                for(auto oaf_itt = outsideAdjacentFaces.begin(); oaf_itt != outsideAdjacentFaces.end(); oaf_itt++)
                {
                    if(af_it->first == (*oaf_itt)->first) { //找到公共外相邻面
                        outsideAdjacentFaces.clear();
                        outsideAdjacentFaces.emplace_back(*oaf_itt);
                        outsideAdjacentFaces.emplace_back(af_it);
                        foundCommonFace = true;
                        break;
                    }
                }
                if(foundCommonFace == true) break; //找到了公共外相邻面
            }
            if(foundCommonFace == true) { //找到了公共外相邻面，继续判断
                for(int i = 1; i < 6; i++)
                {
                    if(i == 2) continue; //跳过第2个侧面(前面已经判断过了)
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //再找剩下4个侧面的外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces[0]->first) continue; //第i个侧面的外相邻面与第0个侧面的外相邻面是同一个面
                        //cout << '#' << faces[i]->first << " 与 #" << af_it->first << " 夹角为" << af_it->second.angle << "， " << af_it->second.concavity << endl;
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个侧面中未找到与第0个侧面外相邻面相同的外相邻面
                }
                if(outsideAdjacentFaces.size() == 6) swap(outsideAdjacentFaces[1], outsideAdjacentFaces[2]); //将第1个侧面的外相邻面与第2个侧面的外相邻面交换，以满足外相邻面按侧面顺序排列
                //六条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces)) outside_typical = true;  //条件符合外部标准
            }
        }

        //若确定为外部标准六角型腔，计算几何参数
        auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //侧相邻面公共边的边曲线索引
        double depth = sideEdge_it->second.length(); //六角型腔的深度
        auto bottomEdge_it = edge_curves.find(concaveAdjacentFaces[0]->second.index); //底面的一个相邻边界
        double length = bottomEdge_it->second.length(); //底面边长

        //判定完毕，输出
        cout << endl << "六角型腔Hexagonal Closed pocket ";
        if(inside_typical == true) cout << "内部标准Typical inside ";
        else cout << "内部非标准Not typical inside ";
        if(outside_typical == true) cout << "外部标准Typical outside: ";
        else cout << "外部非标准Not typical outside: ";
        if(outside_typical == true) cout << "深度depth = " << depth << ", ";
        cout << "底面边长length = " << length;
        cout << " | 底面#" << ad_it->first << ", 六个侧面#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

        //屏蔽关键面
        ad_it->second.setUse(2);
        for(int i = 0; i < 6; i++)
        {
            faces[i]->second.setUse(1);
        }
    }
}
bool reorder6AdjacentFaces(vector<map<int, CommonEdge>::iterator> &concaveAdjacentFaces, vector<map<int, ADVANCED_FACE>::iterator> &faces, vector<CommonEdge*> &sideCommonEdges)
{
    //将六个凹相邻面按顺时针或逆时针顺序依次排列好，并验证每两个相邻面有且仅有一个公共直线边
    for(int i = 0; i < 4; i++)
    {
        bool found = false; //是否找到与第i个面有且仅有一条公共直线边的面
        for(int j = i + 1; j < 6; j++)
        {
            if(faces[i]->second.commonEdge_number_withFace(faces[j]->first) == 1) {
                swap(concaveAdjacentFaces[i + 1], concaveAdjacentFaces[j]);
                swap(faces[i + 1], faces[j]);
                found = true;
                break;
            }
        }
        if(found == false) return false; //没有找到相邻面，返回失败
    }
    if(faces[5]->second.commonEdge_number_withFace(faces[4]->first) != 1) return false;
    if(faces[5]->second.commonEdge_number_withFace(faces[0]->first) != 1) return false;

    //确保不相邻的对位面之间没有公共边
    for(int i = 0; i < 6; i++)
    {
        for(int j = 2; j <= 4; j++)
        {
            if(faces[i]->second.commonEdge_number_withFace(faces[(i + j) % 6]->first) != 0) return false; //与其他三个不相邻面有公共边，返回错误
        }
    }

    //依次返回六个公共直线边指针
    for(int i = 0; i < 6; i++)
    {
        sideCommonEdges.emplace_back(faces[i]->second.getCommonEdge_withAdjacentFace(faces[(i + 1) % 6]->first));
    }

    //验证了六个侧相邻面的相邻关系，完成了面的重新排序和公共直线边的获取后，返回成功
    return true;
}


void squareHole()
{
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        if(ad_it->second.faceType != "PLANE") continue; //不是平面直接跳过

        if(ad_it->second.concaveCommonLineEdge_number() < 2) continue; //凹公共直线边个数必须不少于2个

        //能找到一对夹角90度的凹公共直线边，对应的两个凹相邻面（平面）能找到另一个共同的凹相邻面（平面）、另一对夹角90度的凹公共直线边，过程中要求四条直线边相互平行，每两个相邻面之间有且仅有一条公共直线边
        vector<map<int, ADVANCED_FACE>::iterator> faces; //方孔四个面高级面的迭代器（第0个、第1个面为一对不相邻的对位面，第2个、第3个面为另一对不相邻的对位面）
        vector<CommonEdge*> sideCommonEdges; //四个面两两之间的公共边指针（顺序为：第0个、第2个面之间的公共边，第0个、第3个面之间的公共边，第1个、第2个面之间的公共边，第1个、第3个面之间的公共边）
        bool found = false; //是否找到符合条件的四个面
        for(auto af_it1 = ad_it->second.adjacentFaces.begin(); af_it1 != ad_it->second.adjacentFaces.end(); af_it1++)
        {
            if(advanced_faces.find(af_it1->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
            if(af_it1->second.edgeType != "LINE") continue; //直线公共边
            if(af_it1->second.concavity != 0) continue; //凹公共直线边
            if(!anglesAreEqual(af_it1->second.angle, (PI/2))) continue; //夹角为90度

            for(auto af_it2 = af_it1; af_it2 != ad_it->second.adjacentFaces.end(); af_it2++)
            {
                if(af_it1->first == af_it2->first) continue; //两个凹相邻面不能是同一个面
                if(advanced_faces.find(af_it2->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                if(af_it2->second.edgeType != "LINE") continue; //直线公共边
                if(af_it2->second.concavity != 0) continue; //凹公共直线边
                if(!anglesAreEqual(af_it2->second.angle, (PI/2))) continue; //夹角为90度
                
                if(!isParallel(af_it1->second.getVector(), af_it2->second.getVector())) continue; //两条直线边要相互平行

                if(ad_it->second.commonEdge_number_withFace(af_it1->first) != 1) continue; //与两个凹相邻面分别有且仅有一条公共直线边
                if(ad_it->second.commonEdge_number_withFace(af_it2->first) != 1) continue;

                auto afAd_it1 = advanced_faces.find(af_it1->first); //第一个凹相邻面的高级面迭代器
                auto afAd_it2 = advanced_faces.find(af_it2->first); //第二个凹相邻面的高级面迭代器
                for(auto af_it3 = afAd_it1->second.adjacentFaces.begin(); af_it3 != afAd_it1->second.adjacentFaces.end(); af_it3++)
                {
                    if(advanced_faces.find(af_it3->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                    if(af_it3->second.edgeType != "LINE") continue; //直线公共边
                    if(af_it3->second.concavity != 0) continue; //凹公共直线边
                    if(!anglesAreEqual(af_it3->second.angle, (PI/2))) continue; //夹角为90度
                    for(auto af_it4 = afAd_it2->second.adjacentFaces.begin(); af_it4 != afAd_it2->second.adjacentFaces.end(); af_it4++)
                    {
                        if(af_it3->first != af_it4->first) continue; //两个凹相邻面共同相邻于另一个面
                        if(af_it3->first == ad_it->first) continue; //另一个共同相邻面不能是第一个共同相邻面
                        if(advanced_faces.find(af_it4->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                        if(af_it4->second.edgeType != "LINE") continue; //直线公共边
                        if(af_it4->second.concavity != 0) continue; //凹公共直线边
                        if(!anglesAreEqual(af_it4->second.angle, (PI/2))) continue; //夹角为90度

                        if(!isParallel(af_it3->second.getVector(), af_it4->second.getVector())) continue; //两条直线边要相互平行

                        if(afAd_it1->second.commonEdge_number_withFace(af_it3->first) != 1) continue; //与两个凹相邻面分别有且仅有一条公共直线边
                        if(afAd_it2->second.commonEdge_number_withFace(af_it4->first) != 1) continue;

                        if(!isParallel(af_it1->second.getVector(), af_it3->second.getVector())) continue; //保证四条直线边两两平行

                        //保存方孔四个面的高级面迭代器
                        faces.emplace_back(ad_it);
                        auto afAd_it3 = advanced_faces.find(af_it3->first);
                        faces.emplace_back(afAd_it3);
                        faces.emplace_back(afAd_it1);
                        faces.emplace_back(afAd_it2);
                        
                        //保存四个面两两之间的公共边指针
                        sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it1->first));
                        sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it2->first));
                        sideCommonEdges.emplace_back(afAd_it1->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                        sideCommonEdges.emplace_back(afAd_it2->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                        found = true; //标记已经找到了符合条件的四个面
                        break;
                    }
                    if(found == true) break;
                }
                if(found == true) break;
            }
            if(found == true) break;
        }
        if(found == false) continue; //没有找到符合条件的四个面，跳过

        //四个面不能全是已被识别为特征的面（isUsed == 1），也不能有被屏蔽过的关键面（isUsed == 2）
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

        /*内部标准判定：
        内部四个面都只含有必要边（仅有一个闭环边界）时判定为内部标准，否则为内部非标准。
        （主要是为了区分四个面内有没有开孔凸起等特殊处理）
        */
        bool inside_typical = true; //内部是否标准
        for(int i = 0; i < 4; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //四个面闭环边界不仅一个，内部非标准
        }

        /*外部标准判定：
        四个面同时与同一个平面相邻于一个270度凸直线公共边，且这四条直线边属于并组成一个边环（这里需要一个找边环函数）。
        ————这样的公共外相邻面能找到两个时称为双平面外部标准方孔，可以输出方孔深度；这样的公共外相邻面能只找到一个时称为单平面外部标准方孔，不能输出方孔深度。
        否则外部非标准。        
        */
        bool outside_typical1 = false, outside_typical2 = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces1; //四个面的第一个公共外相邻面
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces2; //四个面的第二个公共外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
            if(af_it->second.edgeType != "LINE") continue; //直线公共边
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //外相邻面是平面
                outsideAdjacentFaces1.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces1.size() == 2) { //对于第0个侧面，符合条件的外相邻面有两个，验证四个侧面分别共同相邻于这两个外相邻面
            outsideAdjacentFaces2.emplace_back(outsideAdjacentFaces1[1]);
            outsideAdjacentFaces1.erase(outsideAdjacentFaces1.begin() + 1);
            //先验证第一个公共外相邻面
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下3个面是否相邻于这个外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //第i个面的外相邻面与第0个面的第一个外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces1.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //第i个面中未找到与第0个面第一个外相邻面相同的外相邻面
            }
            //第一个外相邻面的四条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces1.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //第一个外相邻面条件符合外部标准

            //再验证第二个公共外相邻面
            for(int i = 1; i < 4; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下3个面是否相邻于这个外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces2[0]->first) continue; //第i个面的外相邻面与第0个面的第二个外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces2.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //第i个面中未找到与第0个面第二个外相邻面相同的外相邻面
            }
            //第二个外相邻面的四条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces2.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces2)) outside_typical2 = true; //第二个外相邻面条件符合外部标准

            if(outside_typical2 == true && outside_typical1 == false) { //第二个外相邻面符合外部标准，第一个外相邻面不符合外部标准，交换两个外相邻面
                swap(outsideAdjacentFaces1, outsideAdjacentFaces2);
                swap(outside_typical1, outside_typical2);
            }
        }
        else if(outsideAdjacentFaces1.size() > 2) { //对于第0个侧面，符合条件的外相邻面多于两个，验证四个侧面分别共同相邻于其中的两个外相邻面
            int num = outsideAdjacentFaces1.size(); //第0个侧面的外相邻面个数
            swap(outsideAdjacentFaces1, outsideAdjacentFaces2); //将备选的公共外相邻面放入outsideAdjacentFaces2中备用

            //找第一个公共外相邻面
            while(outside_typical1 != true && num > 0) {
                outsideAdjacentFaces1.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;
                
                for(int i = 1; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下3个面是否相邻于这个外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //第i个面的外相邻面与第0个面的这个外相邻面是同一个面
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces1.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个面中未找到与第0个面的这个外相邻面相同的外相邻面
                }
                //四条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces1.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //此外相邻面条件符合外部标准
                else outsideAdjacentFaces1.clear();
            }
            
            //找到了第一个符合条件的外相邻面，继续找第二个符合条件的外相邻面
            while(outside_typical2 != true && num > 0) {
                vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces3; //临时外相邻面存储器
                outsideAdjacentFaces3.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;

                for(int i = 1; i < 4; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下3个面是否相邻于这个外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces3[0]->first) continue; //第i个面的外相邻面与第0个面的这个外相邻面是同一个面
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces3.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个面中未找到与第0个面的这个外相邻面相同的外相邻面
                }
                //四条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces3.size() == 4 && formAnEdgeLoop(outsideAdjacentFaces3)) { //此外相邻面条件符合外部标准
                    outside_typical2 = true;
                    outsideAdjacentFaces2 = outsideAdjacentFaces3;
                }
            }
        }

        //计算几何参数
        double depth; //方孔深度
        double length; //方孔长度
        double width; //方孔宽度
        if(outside_typical1 == true && outside_typical2 == true) { //若确定为双平面外部标准方孔，可计算方孔深度
            auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //侧面公共边的边曲线索引
            depth = sideEdge_it->second.length(); //方孔的深度
            auto outsideEdge_it1 = edge_curves.find(outsideAdjacentFaces1[0]->second.index); //方孔公共外相邻面的两个相邻边界
            auto outsideEdge_it2 = edge_curves.find(outsideAdjacentFaces1[2]->second.index);
            length = outsideEdge_it1->second.length(); //方孔长度
            width = outsideEdge_it2->second.length(); //方孔宽度
            if(length < width) swap(length, width);
        }
        else if(outside_typical1 == true && outside_typical2 == false) { //单平面外部标准方孔，不可计算方孔深度
            auto outsideEdge_it1 = edge_curves.find(outsideAdjacentFaces1[0]->second.index); //方孔公共外相邻面的两个相邻边界
            auto outsideEdge_it2 = edge_curves.find(outsideAdjacentFaces1[2]->second.index);
            length = outsideEdge_it1->second.length(); //方孔长度
            width = outsideEdge_it2->second.length(); //方孔宽度
            if(length < width) swap(length, width);
        }
        else { //外部非标准方孔，不可计算方孔深度
            auto ec_it_0_2 = edge_curves.find(sideCommonEdges[0]->index); //第0面与第2面公共边的边曲线
            auto ec_it_0_3 = edge_curves.find(sideCommonEdges[1]->index); //第0面与第3面公共边的边曲线
            auto ec_it_1_2 = edge_curves.find(sideCommonEdges[2]->index); //第1面与第2面公共边的边曲线
            auto vp_it_0_2 = vertex_points.find(ec_it_0_2->second.vertex1); //第0面与第2面公共边上的一个顶点
            auto vp_it_0_3 = vertex_points.find(ec_it_0_3->second.vertex1); //第0面与第3面公共边上的一个顶点
            auto vp_it_1_2 = vertex_points.find(ec_it_1_2->second.vertex1); //第1面与第2面公共边上的一个顶点
            auto cp_it_0_2 = cartesian_points.find(vp_it_0_2->second.point); //第0面与第2面公共边上的一个坐标点
            auto cp_it_0_3 = cartesian_points.find(vp_it_0_3->second.point); //第0面与第3面公共边上的一个坐标点
            auto cp_it_1_2 = cartesian_points.find(vp_it_1_2->second.point); //第1面与第2面公共边上的一个坐标点
            Point p_0_2 = cp_it_0_2->second.toPoint(); //第0面与第2面公共边上的一个坐标点
            Point p_0_3 = cp_it_0_3->second.toPoint(); //第0面与第3面公共边上的一个坐标点
            Point p_1_2 = cp_it_1_2->second.toPoint(); //第1面与第2面公共边上的一个坐标点
            Vector v_0_2 = sideCommonEdges[0]->getVector(); //第0面与第2面公共边的方向向量

            Point footOfPerpendicular1 = GetFootOfPerpendicular(p_0_3, p_0_2, v_0_2); //第0面与第3面公共边上的一个坐标点在第0面与第2面公共边上的垂足
            Point footOfPerpendicular2 = GetFootOfPerpendicular(p_1_2, p_0_2, v_0_2); //第1面与第2面公共边上的一个坐标点在第0面与第2面公共边上的垂足
            length = distance_AB(footOfPerpendicular1, p_0_3); //方孔长度
            width = distance_AB(footOfPerpendicular2, p_1_2); //方孔宽度
            if(length < width) swap(length, width);

            // //测试
            // auto ec_it_1_3 = edge_curves.find(sideCommonEdges[3]->index); //第1面与第3面公共边的边曲线
            // auto vp_it_1_3 = vertex_points.find(ec_it_1_3->second.vertex1); //第1面与第3面公共边上的一个顶点
            // auto cp_it_1_3 = cartesian_points.find(vp_it_1_3->second.point); //第1面与第3面公共边上的一个坐标点
            // Point p_1_3 = cp_it_1_3->second.toPoint(); //第1面与第3面公共边上的一个坐标点
            // Vector v_1_3 = sideCommonEdges[3]->getVector(); //第1面与第3面公共边的方向向量
            // Point footOfPerpendicular3 = GetFootOfPerpendicular(p_1_2, p_1_3, v_1_3); //第1面与第2面公共边上的一个坐标点在第1面与第3面公共边上的垂足
            // Point footOfPerpendicular4 = GetFootOfPerpendicular(p_0_3, p_1_3, v_1_3); //第0面与第3面公共边上的一个坐标点在第1面与第3面公共边上的垂足
            // double length1 = distance_AB(footOfPerpendicular3, p_1_2); //方孔长度
            // double width1 = distance_AB(footOfPerpendicular4, p_0_3); //方孔宽度
            // if(length1 < width1) swap(length1, width1);
            // cout << "length1 = " << length1 << ", width1 = " << width1 << endl;
        }

        //判定完毕，输出
        cout << endl << "方孔Square hole ";
        if(inside_typical == true) cout << "内部标准Typical inside ";
        else cout << "内部非标准Not typical inside ";
        if(outside_typical1 == true && outside_typical2 == true) cout << "双平面外部标准Double Typical outside: ";
        else if(outside_typical1 == true && outside_typical2 == false) cout << "单平面外部标准Single Typical outside: ";
        else cout << "外部非标准Not typical outside: ";
        if(outside_typical1 == true && outside_typical2 == true) { //双平面外部标准方孔，输出方孔深度、长度、宽度
            cout << "深度depth = " << depth << ", ";
            cout << "长度length = " << length << ", 宽度width = " << width;
        }
        else if(outside_typical1 == true && outside_typical2 == false) { //单平面外部标准方孔，输出长度、宽度
            cout << "长度length = " << length << ", 宽度width = " << width;
        }
        else { //外部非标准方孔，输出长度、宽度
            cout << "长度length = " << length << ", 宽度width = " << width;
        }
        cout << " | 四个侧面#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << endl;

        //屏蔽关键面
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
        if(ad_it->second.faceType != "PLANE") continue; //不是平面直接跳过

        if(ad_it->second.concaveCommonLineEdge_number() < 2) continue; //凹公共直线边个数必须不少于2个

        //能找到一对夹角120度的凹公共直线边，对应的两个凹相邻面（平面）能分别找到另两个凹相邻面、另一对夹角120度的凹公共直线边，继而再找到一个共同的凹相邻面（平面）、另一对夹角120度的凹公共直线边，过程中要求六条直线边相互平行,每两个相邻面之间有且仅有一条公共直线边
        vector<map<int, ADVANCED_FACE>::iterator> faces; //方孔六个面高级面的迭代器（第0个、第1个、第2个、第3个、第4个、第5个面逐个相邻，首尾相接）
        vector<CommonEdge*> sideCommonEdges; //六个面两两之间的公共边指针（顺序为：第0个、第1个面之间的公共边，第1个、第2个面之间的公共边，第2个、第3个面之间的公共边，第3个、第4个面之间的公共边，第4个、第5个面之间的公共边，第5个、第0个面之间的公共边）
        bool found = false; //是否找到符合条件的六个面
        for(auto af_it1 = ad_it->second.adjacentFaces.begin(); af_it1 != ad_it->second.adjacentFaces.end(); af_it1++)
        {
            if(advanced_faces.find(af_it1->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
            if(af_it1->second.edgeType != "LINE") continue; //直线公共边
            if(af_it1->second.concavity != 0) continue; //凹公共直线边
            if(!anglesAreEqual(af_it1->second.angle, (2*PI/3))) continue; //夹角为120度

            for(auto af_it2 = af_it1; af_it2 != ad_it->second.adjacentFaces.end(); af_it2++)
            {
                if(af_it1->first == af_it2->first) continue; //两个凹相邻面不能是同一个面
                if(advanced_faces.find(af_it2->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                if(af_it2->second.edgeType != "LINE") continue; //直线公共边
                if(af_it2->second.concavity != 0) continue; //凹公共直线边
                if(!anglesAreEqual(af_it2->second.angle, (2*PI/3))) continue; //夹角为120度
                
                if(!isParallel(af_it1->second.getVector(), af_it2->second.getVector())) continue; //两条直线边要相互平行

                if(ad_it->second.commonEdge_number_withFace(af_it1->first) != 1) continue; //与两个凹相邻面分别有且仅有一条公共直线边
                if(ad_it->second.commonEdge_number_withFace(af_it2->first) != 1) continue;

                auto afAd_it1 = advanced_faces.find(af_it1->first); //第一个凹相邻面的高级面迭代器
                auto afAd_it2 = advanced_faces.find(af_it2->first); //第二个凹相邻面的高级面迭代器
                for(auto af_it3 = afAd_it1->second.adjacentFaces.begin(); af_it3 != afAd_it1->second.adjacentFaces.end(); af_it3++)
                {
                    if(af_it3->first == ad_it->first) continue; //第一个凹相邻面的另一个相邻面不能是起始面
                    if(advanced_faces.find(af_it3->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                    if(af_it3->second.edgeType != "LINE") continue; //直线公共边
                    if(af_it3->second.concavity != 0) continue; //凹公共直线边
                    if(!anglesAreEqual(af_it3->second.angle, (2*PI/3))) continue; //夹角为120度

                    if(!isParallel(af_it1->second.getVector(), af_it3->second.getVector())) continue; //两条直线边要相互平行

                    if(afAd_it1->second.commonEdge_number_withFace(af_it3->first) != 1) continue; //与前一个相邻面有且仅有一条公共直线边

                    for(auto af_it4 = afAd_it2->second.adjacentFaces.begin(); af_it4 != afAd_it2->second.adjacentFaces.end(); af_it4++)
                    {
                        if(af_it4->first == ad_it->first) continue; //第二个凹相邻面的另一个相邻面不能是起始面
                        if(advanced_faces.find(af_it4->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                        if(af_it4->second.edgeType != "LINE") continue; //直线公共边
                        if(af_it4->second.concavity != 0) continue; //凹公共直线边
                        if(!anglesAreEqual(af_it4->second.angle, (2*PI/3))) continue; //夹角为120度

                        if(!isParallel(af_it2->second.getVector(), af_it4->second.getVector())) continue; //两条直线边要相互平行

                        if(afAd_it2->second.commonEdge_number_withFace(af_it4->first) != 1) continue; //与前一个相邻面有且仅有一条公共直线边

                        auto afAd_it3 = advanced_faces.find(af_it3->first); //第三个凹相邻面的高级面迭代器
                        auto afAd_it4 = advanced_faces.find(af_it4->first); //第四个凹相邻面的高级面迭代器
                        for(auto af_it5 = afAd_it3->second.adjacentFaces.begin(); af_it5 != afAd_it3->second.adjacentFaces.end(); af_it5++)
                        {
                            if(af_it5->first == afAd_it1->first) continue; //第五个凹相邻面的另一个相邻面不能是第一个凹相邻面
                            if(advanced_faces.find(af_it5->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                            if(af_it5->second.edgeType != "LINE") continue; //直线公共边
                            if(af_it5->second.concavity != 0) continue; //凹公共直线边
                            if(!anglesAreEqual(af_it5->second.angle, (2*PI/3))) continue; //夹角为120度

                            if(!isParallel(af_it3->second.getVector(), af_it5->second.getVector())) continue; //两条直线边要相互平行

                            if(afAd_it3->second.commonEdge_number_withFace(af_it5->first) != 1) continue; //与前一个相邻面有且仅有一条公共直线边
                            for(auto af_it6 = afAd_it4->second.adjacentFaces.begin(); af_it6 != afAd_it4->second.adjacentFaces.end(); af_it6++)
                            {
                                if(af_it5->first != af_it6->first) continue; //第五和第六个凹相邻面是同一个面
                                if(af_it6->first == afAd_it2->first) continue; //第六个凹相邻面的另一个相邻面不能是第二个凹相邻面
                                if(advanced_faces.find(af_it6->first)->second.faceType != "PLANE") continue; //不是平面直接跳过
                                if(af_it6->second.edgeType != "LINE") continue; //直线公共边
                                if(af_it6->second.concavity != 0) continue; //凹公共直线边
                                if(!anglesAreEqual(af_it6->second.angle, (2*PI/3))) continue; //夹角为120度

                                if(!isParallel(af_it2->second.getVector(), af_it6->second.getVector())) continue; //两条直线边要相互平行

                                if(afAd_it4->second.commonEdge_number_withFace(af_it6->first) != 1) continue; //与前一个相邻面有且仅有一条公共直线边
                                
                                //保存六角孔的六个面的高级面迭代器
                                faces.emplace_back(ad_it);
                                faces.emplace_back(afAd_it1);
                                faces.emplace_back(afAd_it3);
                                auto afAd_it5 = advanced_faces.find(af_it5->first);
                                faces.emplace_back(afAd_it5);
                                faces.emplace_back(afAd_it4);
                                faces.emplace_back(afAd_it2);

                                //保存六角孔的六个面两两之间的公共边指针
                                sideCommonEdges.emplace_back(ad_it->second.getCommonEdge_withAdjacentFace(afAd_it1->first));
                                sideCommonEdges.emplace_back(afAd_it1->second.getCommonEdge_withAdjacentFace(afAd_it3->first));
                                sideCommonEdges.emplace_back(afAd_it3->second.getCommonEdge_withAdjacentFace(afAd_it5->first));
                                sideCommonEdges.emplace_back(afAd_it5->second.getCommonEdge_withAdjacentFace(afAd_it4->first));
                                sideCommonEdges.emplace_back(afAd_it4->second.getCommonEdge_withAdjacentFace(afAd_it2->first));
                                sideCommonEdges.emplace_back(afAd_it2->second.getCommonEdge_withAdjacentFace(ad_it->first));
                                found = true; //标记已经找到了符合条件的六个面
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
        if(found == false) continue; //未找到符合条件的六个面，跳过

        //六个面不能全是已被识别为特征的面（isUsed == 1）,也不能有被屏蔽过的关键面（isUsed == 2）
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

        /*内部标准判定：
        内部六个面都只含有必要边（仅有一个闭环边界）时判定为内部标准，否则为内部非标准。
        （主要是为了区分六个面内有没有开孔凸起等特殊处理）
        */
        bool inside_typical = true; //内部标准
        for(int i = 0; i < 6; i++)
        {
            if(faces[i]->second.bounds.size() != 1) inside_typical = false; //六个面闭环边界不仅一个，内部非标准
        }

        /*外部标准判定：
        六个面同时与同一个平面相邻于一个270度凸直线公共边，且这六条直线边属于并组成一个边环（这里需要一个找边环函数）。
        ————这样的公共外相邻面能找到两个时称为双平面外部标准六角孔，可以输出六角孔深度；这样的公共外相邻面能只找到一个时称为单平面外部标准六角孔，不能输出六角孔深度。
        否则外部非标准。
        */
        bool outside_typical1 = false, outside_typical2 = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces1; //六个面的第一个公共外相邻面
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces2; //六个面的第二个公共外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
            if(af_it->second.edgeType != "LINE") continue; //直线公共边
            auto oaf_it = advanced_faces.find(af_it->first);
            if(planes.find(oaf_it->second.face) != planes.end()) { //外相邻面是平面
                outsideAdjacentFaces1.emplace_back(af_it);
            }
        }
        if(outsideAdjacentFaces1.size() == 2) { //对于第0个侧面，符合条件的外相邻面有两个，验证六个侧面分别共同相邻于这两个外相邻面
            outsideAdjacentFaces2.emplace_back(outsideAdjacentFaces1[1]);
            outsideAdjacentFaces1.erase(outsideAdjacentFaces1.begin() + 1);
            //先验证第一个公共外相邻面
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下5个面是否相邻于这个外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //第i个面的外相邻面与第0个面的第一个外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces1.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //第i个面中未找到与第0个面的第一个外相邻面相同的外相邻面
            }
            //第一个外相邻面的六条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces1.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //第一个外相邻面条件符合外部标准

            //再验证第二个公共外相邻面
            for(int i = 1; i < 6; i++)
            {
                bool found = false;
                for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下5个面是否相邻于这个外相邻面
                {
                    if(af_it->first != outsideAdjacentFaces2[0]->first) continue; //第i个面的外相邻面与第0个面的第二个外相邻面是同一个面
                    if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                    if(af_it->second.concavity != 1) continue; //凸
                    if(af_it->second.edgeType != "LINE") continue; //直线公共边
                    outsideAdjacentFaces2.emplace_back(af_it);
                    found = true;
                    break;
                }
                if(found == false) break; //第i个面中未找到与第0个面的第二个外相邻面相同的外相邻面
            }
            //第二个外相邻面的六条外相邻直线边属于并组成一个边环
            if(outsideAdjacentFaces2.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces2)) outside_typical2 = true; //第二个外相邻面条件符合外部标准

            if(outside_typical2 == true && outside_typical1 == false) { //第二个外相邻面符合外部标准，第一个外相邻面不符合外部标准，交换两个外相邻面
                swap(outsideAdjacentFaces1, outsideAdjacentFaces2);
                swap(outside_typical1, outside_typical2);
            }
        }
        else if(outsideAdjacentFaces1.size() > 2) { //对于第0个侧面，符合条件的外相邻面有多于两个，验证六个侧面分别共同相邻于其中的两个外相邻面
            int num = outsideAdjacentFaces1.size(); //第0个侧面的外相邻面个数
            swap(outsideAdjacentFaces1, outsideAdjacentFaces2); //将备选的公共外相邻面放入outsideAdjacentFaces2中备用

            //找第一个公共外相邻面
            while(outside_typical1 != true && num > 0) {
                outsideAdjacentFaces1.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;
                
                for(int i = 1; i < 6; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下5个面是否相邻于这个外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces1[0]->first) continue; //第i个面的外相邻面与第0个面的这个外相邻面是同一个面
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces1.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个面中未找到与第0个面的这个外相邻面相同的外相邻面
                }
                //六条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces1.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces1)) outside_typical1 = true; //此外相邻面条件符合外部标准
                else outsideAdjacentFaces1.clear();
            }

            //找到了第一个符合条件的外相邻面，继续找第二个符合条件的外相邻面
            while(outside_typical2 != true && num > 0) {
                vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces3; //临时外相邻面存储器
                outsideAdjacentFaces3.emplace_back(outsideAdjacentFaces2[num - 1]);
                outsideAdjacentFaces2.erase(outsideAdjacentFaces2.begin() + num - 1);
                num--;

                for(int i = 1; i < 6; i++)
                {
                    bool found = false;
                    for(auto af_it = faces[i]->second.adjacentFaces.begin(); af_it != faces[i]->second.adjacentFaces.end(); af_it++) //验证剩下5个面是否相邻于这个外相邻面
                    {
                        if(af_it->first != outsideAdjacentFaces3[0]->first) continue; //第i个面的外相邻面与第0个面的这个外相邻面是同一个面
                        if(!anglesAreEqual(af_it->second.angle, (3*PI/2))) continue; //夹角为270度
                        if(af_it->second.concavity != 1) continue; //凸
                        if(af_it->second.edgeType != "LINE") continue; //直线公共边
                        outsideAdjacentFaces3.emplace_back(af_it);
                        found = true;
                        break;
                    }
                    if(found == false) break; //第i个面中未找到与第0个面的这个外相邻面相同的外相邻面
                }
                //六条外相邻直线边属于并组成一个边环
                if(outsideAdjacentFaces3.size() == 6 && formAnEdgeLoop(outsideAdjacentFaces3)) { //此外相邻面条件符合外部标准
                    outside_typical2 = true;
                    outsideAdjacentFaces2 = outsideAdjacentFaces3;
                }
            }

            //计算几何参数
            double depth; //六角孔深度
            vector<double> length; //六角孔边长

            auto ec_it_0_1 = edge_curves.find(sideCommonEdges[0]->index); //第0面与第1面公共边的边曲线
            auto ec_it_1_2 = edge_curves.find(sideCommonEdges[1]->index); //第1面与第2面公共边的边曲线
            auto ec_it_2_3 = edge_curves.find(sideCommonEdges[2]->index); //第2面与第3面公共边的边曲线
            auto ec_it_3_4 = edge_curves.find(sideCommonEdges[3]->index); //第3面与第4面公共边的边曲线
            auto ec_it_4_5 = edge_curves.find(sideCommonEdges[4]->index); //第4面与第5面公共边的边曲线
            auto ec_it_5_0 = edge_curves.find(sideCommonEdges[5]->index); //第5面与第0面公共边的边曲线
            auto vp_it_0_1 = vertex_points.find(ec_it_0_1->second.vertex1); //第0面与第1面公共边上的一个顶点
            auto vp_it_1_2 = vertex_points.find(ec_it_1_2->second.vertex1); //第1面与第2面公共边上的一个顶点
            auto vp_it_2_3 = vertex_points.find(ec_it_2_3->second.vertex1); //第2面与第3面公共边上的一个顶点
            auto vp_it_3_4 = vertex_points.find(ec_it_3_4->second.vertex1); //第3面与第4面公共边上的一个顶点
            auto vp_it_4_5 = vertex_points.find(ec_it_4_5->second.vertex1); //第4面与第5面公共边上的一个顶点
            auto vp_it_5_0 = vertex_points.find(ec_it_5_0->second.vertex1); //第5面与第0面公共边上的一个顶点
            auto cp_it_0_1 = cartesian_points.find(vp_it_0_1->second.point); //第0面与第1面公共边上的一个坐标点
            auto cp_it_1_2 = cartesian_points.find(vp_it_1_2->second.point); //第1面与第2面公共边上的一个坐标点
            auto cp_it_2_3 = cartesian_points.find(vp_it_2_3->second.point); //第2面与第3面公共边上的一个坐标点
            auto cp_it_3_4 = cartesian_points.find(vp_it_3_4->second.point); //第3面与第4面公共边上的一个坐标点
            auto cp_it_4_5 = cartesian_points.find(vp_it_4_5->second.point); //第4面与第5面公共边上的一个坐标点
            auto cp_it_5_0 = cartesian_points.find(vp_it_5_0->second.point); //第5面与第0面公共边上的一个坐标点
            Point p_0_1 = cp_it_0_1->second.toPoint(); //第0面与第1面公共边上的一个坐标点
            Point p_1_2 = cp_it_1_2->second.toPoint(); //第1面与第2面公共边上的一个坐标点
            Point p_2_3 = cp_it_2_3->second.toPoint(); //第2面与第3面公共边上的一个坐标点
            Point p_3_4 = cp_it_3_4->second.toPoint(); //第3面与第4面公共边上的一个坐标点
            Point p_4_5 = cp_it_4_5->second.toPoint(); //第4面与第5面公共边上的一个坐标点
            Point p_5_0 = cp_it_5_0->second.toPoint(); //第5面与第0面公共边上的一个坐标点
            Vector v_0_1 = sideCommonEdges[0]->getVector(); //第0面与第1面公共边的方向向量
            Vector v_1_2 = sideCommonEdges[1]->getVector(); //第1面与第2面公共边的方向向量
            Vector v_2_3 = sideCommonEdges[2]->getVector(); //第2面与第3面公共边的方向向量
            Vector v_3_4 = sideCommonEdges[3]->getVector(); //第3面与第4面公共边的方向向量
            Vector v_4_5 = sideCommonEdges[4]->getVector(); //第4面与第5面公共边的方向向量
            Vector v_5_0 = sideCommonEdges[5]->getVector(); //第5面与第0面公共边的方向向量

            Point footOfPerpendicular1 = GetFootOfPerpendicular(p_1_2, p_0_1, v_0_1); //第1面与第2面公共边上的一个坐标点在第0面与第1面公共边上的垂足
            Point footOfPerpendicular2 = GetFootOfPerpendicular(p_2_3, p_1_2, v_1_2); //第2面与第3面公共边上的一个坐标点在第1面与第2面公共边上的垂足
            Point footOfPerpendicular3 = GetFootOfPerpendicular(p_3_4, p_2_3, v_2_3); //第3面与第4面公共边上的一个坐标点在第2面与第3面公共边上的垂足
            Point footOfPerpendicular4 = GetFootOfPerpendicular(p_4_5, p_3_4, v_3_4); //第4面与第5面公共边上的一个坐标点在第3面与第4面公共边上的垂足
            Point footOfPerpendicular5 = GetFootOfPerpendicular(p_5_0, p_4_5, v_4_5); //第5面与第0面公共边上的一个坐标点在第4面与第5面公共边上的垂足
            Point footOfPerpendicular6 = GetFootOfPerpendicular(p_0_1, p_5_0, v_5_0); //第0面与第1面公共边上的一个坐标点在第5面与第0面公共边上的垂足
            length.emplace_back(distance_AB(footOfPerpendicular1, p_1_2)); //第1面的边长
            length.emplace_back(distance_AB(footOfPerpendicular2, p_2_3)); //第2面的边长
            length.emplace_back(distance_AB(footOfPerpendicular3, p_3_4)); //第3面的边长
            length.emplace_back(distance_AB(footOfPerpendicular4, p_4_5)); //第4面的边长
            length.emplace_back(distance_AB(footOfPerpendicular5, p_5_0)); //第5面的边长
            length.emplace_back(distance_AB(footOfPerpendicular6, p_0_1)); //第0面的边长
            sort(length.begin(), length.end()); //将边长从小到大排序
            bool isRegular = true; //是不是正六角孔
            for(int i = 1; i < 6; i++)
            {
                if(!isEqual(length[i], length[0])) isRegular = false; //边长不相等，不是正六角孔
            }
            
            if(outside_typical1 == true && outside_typical2 == true) { //双平面外部标准六角孔，可以计算六角孔深度
                auto sideEdge_it = edge_curves.find(sideCommonEdges[0]->index); //侧面公共边的边曲线索引
                depth = sideEdge_it->second.length(); //六角孔深度
            }

            //判定完毕，输出
            if(isRegular == true) cout << endl << "正六角孔Regular Hexagonal hole ";
            else cout << endl << "六角孔Hexagonal hole ";
            if(inside_typical == true) cout << "内部标准Typical inside ";
            else cout << "内部非标准Not typical inside ";
            if(outside_typical1 == true && outside_typical2 == true) cout << "双平面外部标准Double Typical outside: ";
            else if(outside_typical1 == true && outside_typical2 == false) cout << "单平面外部标准Single Typical outside: ";
            else cout << "外部非标准Not typical outside: ";
            if(outside_typical1 == true && outside_typical2 == true) { //双平面外部标准六角孔，输出六角孔深度
                cout << "深度depth = " << depth << ", ";
            }
            if(isRegular == true) { //正六角孔，输出六角孔边长
                cout << "边长length = " << length[0];
            }
            else { //非正六角孔，输出六角孔边长
                cout << "边长length = ";
                for(int i = 0; i < 6; i++)
                {
                    cout << length[i];
                    if(i != 5) cout << ", ";
                }
            }
            cout << " | 六个侧面#" << faces[0]->first << ' ' << faces[1]->first << ' ' << faces[2]->first << ' ' << faces[3]->first << ' ' << faces[4]->first << ' ' << faces[5]->first << endl;

            //屏蔽关键面
            for(int i = 0; i < 6; i++)
            {
                faces[i]->second.setUse(1);
            }
        }
    }
}