#include "LexAnalysis.h"

void output()
{
    cout <<endl;
    for(auto i = closed_shells.begin(); i != closed_shells.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = advanced_faces.begin(); i != advanced_faces.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = face_outer_bounds.begin(); i != face_outer_bounds.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = face_bounds.begin(); i != face_bounds.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = planes.begin(); i != planes.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = cylindrical_surfaces.begin(); i != cylindrical_surfaces.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = conical_surfaces.begin(); i != conical_surfaces.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = spherical_surfaces.begin(); i != spherical_surfaces.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = toroidal_surfaces.begin(); i != toroidal_surfaces.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = axis2_pacement_3ds.begin(); i != axis2_pacement_3ds.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = edge_loops.begin(); i != edge_loops.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = oriented_edges.begin(); i != oriented_edges.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = edge_curves.begin(); i != edge_curves.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = vertex_points.begin(); i != vertex_points.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = cartesian_points.begin(); i != cartesian_points.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = lines.begin(); i != lines.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = vectors.begin(); i != vectors.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = directions.begin(); i != directions.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
    for(auto i = circles.begin(); i != circles.end(); i++)
    {
        cout << i->second.index << ' ';
    }
    cout <<endl;
}
int main()
{
    string filepath;
    while(cin >> filepath)
    {
        cout << readSTPtoCPP(filepath) << endl;
        output();
    }
    return 0;
}