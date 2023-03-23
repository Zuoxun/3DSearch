#include "LexAnalysis.h"
#include "LexAnalysis.cpp"
#include "FeatureAnalysis.h"
#include "FeatureAnalysis.cpp"

void test()
{
    cout << PI <<endl;
    cout << __GNUC__ << endl;
    cout << __cplusplus << endl;
    // auto ajf_it = advanced_faces.begin()->second.adjacentFaces.begin();
    // cout << "test " << ajf_it->first << ',' << ajf_it->second.begin()->first << ',' << ajf_it->second.begin()->second << endl;
    // map<int ,int> temp;
    // temp.emplace(123, -1);
    // temp.emplace(321, 0);
    // advanced_faces.begin()->second.adjacentFaces.emplace(12345, temp);
    // ajf_it = advanced_faces.begin()->second.adjacentFaces.begin();
    // auto it = ajf_it->second.begin();
    // cout << "test " << ajf_it->first  << ',' << it->first << ',' << it->second << endl;
    // it++;
    // cout << "test " << ajf_it->first << ',' << it->first << ',' << it->second << endl;
    // cout << ajf_it->second.size() << endl;

    //求点到射线垂向量测试
    Point pt = {0, 0, 0};
    auto cy_it = cylindrical_surfaces.begin();
    auto ad_it = advanced_faces.find(cy_it->second.upIndex);
    auto ax_it = axis2_pacement_3ds.find(cy_it->second.axis2);
    auto cp_it = cartesian_points.find(ax_it->second.point);
    Point begin = cp_it->second.toPoint();
    Vector vec = ad_it->second.getDirectionZ();
    Point foot = GetFootOfPerpendicular(pt, begin, vec);
    Vector Perpendicular = GetPerpendicular(pt, begin, vec);
    cout << "test foot(" << foot.x << ',' << foot.y << ',' << foot.z << ')' << endl;
    cout << "test pt(" << pt.x << ',' << pt.y << ',' << pt.z << ") " << "begin(" << begin.x << ',' << begin.y << ',' << begin.z << ") " << "vec(" << vec.x << ',' << vec.y << ',' << vec.z << ") " << "Perpendicular(" << Perpendicular.x << ',' << Perpendicular.y << ',' << Perpendicular.z << ')' << endl;

    // Vector v1 = {1, 1, 1};
    // Vector v2 = {-1, 1, 1};
    // Vector v3 = v1 * v2;
    // cout << "v1*v2=v3" << '(' << v3.x << ',' << v3.y << ',' << v3.z << ')' << endl;
    // Vector v1 = {-1, 1, 0};
    // Vector v2 = {-1, 0, 0};
    // cout << "Angle = " << getAngle(v1, v2) << endl;

    Point pt1 = {1, 1, 1};
    Point begin1 = {2, 2, 2};
    Vector vec1 = {-1, -1, 1};
    Point foot1 = GetFootOfPerpendicular(pt1, begin1, vec1);
    Vector Perpendicular1 = GetPerpendicular(pt1, begin1, vec1);
    cout << "test foot1(" << foot1.x << ',' << foot1.y << ',' << foot1.z << ')' << endl;
    cout << "test Perpendicular1(" << Perpendicular1.x << ',' << Perpendicular1.y << ',' << Perpendicular1.z << ')' << endl;
}

void face_use_visit()
{
    cout << endl << "face_use_visit:" << endl;
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        cout << ad_it->second.index << ".isUsed = " << ad_it->second.isUsed << ", .isVisited = " << ad_it->second.isVisited << endl;
    }
}


int main()
{
    string filepath;
    while(getline(cin, filepath))
    {
        clearData();
        if(readSTPtoCPP(filepath)) {
            cout << "ReadSTPtoCPP succeeded!" << endl;
            makeIndexTable();
            makeAAG();
            output();
            cylinder_and_circularHole();
            cone_and_conicalHole();
            setUse(0);
            closedPocket();
            hexagonalClosedPocket();
        }
        else cout << "ReadSTPtoCPP failed!" << endl;
        //test();
    }
    return 0;
}
