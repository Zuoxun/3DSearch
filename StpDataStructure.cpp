#include "StpDataStructure.h"

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

string FACE_OUTER_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}

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
bool FACE_OUTER_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}

bool FACE_OUTER_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}

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

string FACE_BOUND::indexType(int index)
{
    if(vertex_loops.find(index) != vertex_loops.end()) return "VERTEX_LOOP";
    else if(edge_loops.find(index) != edge_loops.end()) return "EDGE_LOOP";
    return "";
}

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

bool FACE_BOUND::isSingleEdge()
{
    auto lp_it = edge_loops.find(loop);
    if(lp_it == edge_loops.end()) return false;
    if(lp_it->second.edges.size() == 1) {
        return true;
    }
    return false;
}

bool FACE_BOUND::isVERTEX()
{
    if(vertex_loops.find(loop) != vertex_loops.end()) return true;
    return false;
}

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

string DIRECTION::indexType(int index)
{
    if(vectors.find(index) != vectors.end()) return "VECTOR";
    else if(axis2_pacement_3ds.find(index) != axis2_pacement_3ds.end()) return "AXIS2_PLACEMENT_3D";
    return "";
}