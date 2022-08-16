#include "LexAnalysis.h"

void output()
{
    cout << "CLOSED_SHELL" << endl;
    for(auto i = closed_shells.begin(); i != closed_shells.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text <<" faces=";
        for(auto j = i->second.faces.begin(); j != i->second.faces.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << endl;
    }
    cout << "ADVANCED_FACE" << endl;
    for(auto i = advanced_faces.begin(); i != advanced_faces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " bounds=";
        for(auto j = i->second.bounds.begin(); j != i->second.bounds.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << " face=" << i->second.face << " flag=" << i->second.flag;
        cout << endl;
    }
    cout << "FACE_OUTER_BOUND" << endl;
    for(auto i = face_outer_bounds.begin(); i != face_outer_bounds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " loop=" << i->second.loop << " flag=" << i->second.flag;
        cout << endl;
    }
    cout << "FACE_BOUND" << endl;
    for(auto i = face_bounds.begin(); i != face_bounds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " loop=" << i->second.loop << " flag=" << i->second.flag;
        cout << endl;
    }
    cout << "PLANE" << endl;
    for(auto i = planes.begin(); i != planes.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2;
        cout << endl;
    }
    cout << "CYLINDRICAL_SURFACE" << endl;
    for(auto i = cylindrical_surfaces.begin(); i != cylindrical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << endl;
    }
    cout << "CONICAL_SURFACE" << endl;
    for(auto i = conical_surfaces.begin(); i != conical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius << " coneAngle=" << i->second.coneAngle;
        cout << endl;
    }
    cout << "SPHERICAL_SURFACE" << endl;
    for(auto i = spherical_surfaces.begin(); i != spherical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << endl;
    }
    cout << "TOROIDAL_SURFACE" << endl;
    for(auto i = toroidal_surfaces.begin(); i != toroidal_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " major_radius=" << i->second.major_radius << " minor_radius=" << i->second.minor_radius;
        cout << endl;
    }
    cout << "AXIS2_PLACEMENT_3D" << endl;
    for(auto i = axis2_pacement_3ds.begin(); i != axis2_pacement_3ds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point << " directionZ=" << i->second.directionZ << " directionX=" << i->second.directionX;
        cout << endl;
    }
    cout << "EDGE_LOOP" << endl;
    for(auto i = edge_loops.begin(); i != edge_loops.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text <<" edges=";
        for(auto j = i->second.edges.begin(); j != i->second.edges.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << endl;
    }
    cout << "ORIENTED_EDGE" << endl;
    for(auto i = oriented_edges.begin(); i != oriented_edges.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " curve=" << i->second.curve << " flag=" << i->second.flag;
        cout << endl;
    }
    cout << "EDGE_CURVE" << endl;
    for(auto i = edge_curves.begin(); i != edge_curves.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " vertex1=" << i->second.vertex1 << " vertex2=" << i->second.vertex2 << " edge=" << i->second.edge << " flag=" << i->second.flag;
        cout << endl;
    }
    cout << "VERTEX_POINT" << endl;
    for(auto i = vertex_points.begin(); i != vertex_points.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point;
        cout << endl;
    }
    cout << "CARTESIAN_POINT" << endl;
    for(auto i = cartesian_points.begin(); i != cartesian_points.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " (x,y,z)=" << i->second.x << ", " << i->second.y << ", " << i->second.z;
        cout << endl;
    }
    cout << "LINE" << endl;
    for(auto i = lines.begin(); i != lines.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point << " vector=" << i->second.vector;
        cout << endl;
    }
    cout << "VECTOR" << endl;
    for(auto i = vectors.begin(); i != vectors.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " direction=" << i->second.direction << " n=" << i->second.n;
        cout << endl;
    }
    cout << "DIRECTION" << endl;
    for(auto i = directions.begin(); i != directions.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " (x,y,z)=" << i->second.x << ", " << i->second.y << ", " << i->second.z;
        cout << endl;
    }
    cout << "CIRCLE" << endl;
    for(auto i = circles.begin(); i != circles.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << endl;
    }
}
int main()
{
    string filepath;
    while(getline(cin, filepath))
    {
        if(readSTPtoCPP(filepath)) {
            cout << "ReadSTPtoCPP succeeded!" << endl;
        }
        else cout << "ReadSTPtoCPP failed!" << endl;
        output();
    }
    return 0;
}
