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

                    double theta = getAngle(N, N1); // N 与 N1 的夹角θ
                    int concavity; //凹凸性
                    if(fabs(theta) > PI / 2) concavity = 1; // |θ| > π/2时为凸
                    else if(fabs(theta) < PI / 2) concavity = 0; // |θ| < π/2时为凹
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

        if(ad_it->second.concaveCommonLineEdge_number() < 4) continue; //凹公共直线边个数必须多于4个
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
        四个侧面同时与同一个平面（除了底面）相邻于一个90度凸直线公共边，且这四条直线边属于并组成一个边环（这里需要一个找边环函数），且四条边相邻垂直。——外部标准，可以输出腔深度
        否则外部非标准。
        */
        bool outside_typical = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //四个侧面的外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个侧面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //夹角为90度
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
                    if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //夹角为90度
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

        if(ad_it->second.concaveCommonLineEdge_number() < 6) continue; //凹公共直线边个数必须多于6个

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
        六个侧面同时与同一个平面（除了底面）相邻于一个90度凸直线公共边，且这六条直线边属于并组成一个边环（这里需要一个找边环函数），且六条边相邻垂直。——外部标准，可以输出腔深度
        否则外部非标准。
        */
        bool outside_typical = false; //外部是否标准
        vector<map<int, CommonEdge>::iterator> outsideAdjacentFaces; //六个侧面的外相邻面
        for(auto af_it = faces[0]->second.adjacentFaces.begin(); af_it != faces[0]->second.adjacentFaces.end(); af_it++) //先找到第0个侧面的外相邻面
        {
            if(af_it->second.concavity != 1) continue; //凸
            if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //夹角为90度
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
                    if(anglesAreEqual(af_it->second.angle, (PI/2))) continue; //夹角为90度
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