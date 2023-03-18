#include "LexAnalysis.h"

void clearData()
{
    closed_shells.clear();
    advanced_faces.clear();
    face_outer_bounds.clear();
    face_bounds.clear();
    planes.clear();
    cylindrical_surfaces.clear();
    conical_surfaces.clear();
    spherical_surfaces.clear();
    toroidal_surfaces.clear();
    axis2_pacement_3ds.clear();
    vertex_loops.clear();
    edge_loops.clear();
    oriented_edges.clear();
    edge_curves.clear();
    vertex_points.clear();
    cartesian_points.clear();
    lines.clear();
    vectors.clear();
    directions.clear();
    circles.clear();
}

string readIndex(fstream &f)
{
    string index;
    char ch;
    while('=' != (ch = f.get())) {
        if(ch == ' ') continue; //屏蔽空格
        else {
            index += ch;
        }
    }
    return index;
}

string readKeyword(fstream &f)
{
    string keyword;
    char ch;
    while('(' != (ch = f.get())) {
        if(ch == ' ') continue; //屏蔽空格
        else {
            keyword += ch;
        }
    }
    return keyword;
}

string readContent(fstream &f)
{
    string content;
    char ch;
    while(';' != (ch = f.get())) {
        if(ch == ' ') continue; //屏蔽空格(故text中若有空格会被删除)
        else {
            content += ch;
        }
    }
    return content;
}

void ignoreContent(fstream &f)
{
    char ch;
    while(';' != (ch = f.get())) {} //省略到语句结束的';'字符
}

bool isKeyword(string keyword)
{
    if(keywords.find(keyword) != keywords.end()) return true;
    else return false;
}

double toDouble(string str)
{
    double num;
    int exponent; //科学计数法10的指数
    int i = 0;
    while(str[i] != 'E' && i < str.size()) {
        i++;
    }
    num = stod(str.substr(0, i)); //除去E指数部分的浮点数部分
    if(str[i] == 'E') {
        exponent = stoi(str.substr(i+1, str.size() - (i+1)));
        while(exponent < 0) {
            num /= 10.0;
            exponent++;
        }
        while(exponent > 0) {
            num *= 10.0;
            exponent--;
        }
    }
    return num;
}

bool isNumber(char ch)
{
    if(ch >= '0' && ch <= '9') return true;
    else return false;
}

string getText(string content, int &i)
{
    string s;
    while(content[i++] != '\'') {} //定位到第一个'字符后
    while(content[i] != '\'') {
        s += content[i++];
    }
    return s;
}

void getIndexGroup(vector<int> &indexGroup, string content, int &i)
{
    string s;
    while(content[i] != ')') {
        if(content[i] == '#') {
            s.clear();
            i++;
            while (isNumber(content[i])) {
                s += content[i++];
            }
            indexGroup.emplace_back(atoi(s.c_str()));
        }
        else i++;
    }
}

int getAnIndex(string content, int &i)
{
    string s;
    while(content[i++] != '#') {}
    while(isNumber(content[i])) {
        s += content[i++];
    }
    return atoi(s.c_str());
}

double getDouble(string content, int &i)
{
    string s;
    while(!isNumber(content[i]) && content[i] != '-') {
        i++;
    }
    while(content[i] != ')' && content[i] != ',') {
        s += content[i++];
    }
    return toDouble(s);
}

bool getFlag(string content, int &i)
{
    while(content[i] != 'T' && content[i] != 'F') {i++;}
    return (content[i] == 'T') ? true : false;
}

void instantiation(string index, string keyword, string content)
{
    if(keyword == "CLOSED_SHELL") {
        CLOSED_SHELL temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //faces高级面
        getIndexGroup(temp.faces, content, i);

        closed_shells.emplace(temp.index, temp);
    }
    else if(keyword == "ADVANCED_FACE") {
        ADVANCED_FACE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //bounds面边界
        getIndexGroup(temp.bounds, content, i);
        //face面
        temp.face = getAnIndex(content, i);
        //flag = T/F
        temp.flag = getFlag(content, i);

        advanced_faces.emplace(temp.index, temp);
    }
    else if(keyword == "FACE_OUTER_BOUND") {
        FACE_OUTER_BOUND temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //loop边环
        temp.loop = getAnIndex(content, i);
        //flag = T/F
        temp.flag = getFlag(content, i);

        face_outer_bounds.emplace(temp.index, temp);
    }
    else if(keyword == "FACE_BOUND") {
        FACE_BOUND temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //loop边环
        temp.loop = getAnIndex(content, i);
        //flag = T/F
        temp.flag = getFlag(content, i);

        face_bounds.emplace(temp.index, temp);
    }
    else if(keyword == "PLANE") {
        PLANE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);

        planes.emplace(temp.index, temp);
    }
    else if(keyword == "CYLINDRICAL_SURFACE") {
        CYLINDRICAL_SURFACE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);
        //radius半径
        temp.radius = getDouble(content, i);

        cylindrical_surfaces.emplace(temp.index, temp);
    }
    else if(keyword == "CONICAL_SURFACE") {
        CONICAL_SURFACE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);
        //radius半径
        temp.radius = getDouble(content, i);
        //coneAngle圆锥角
        temp.coneAngle = getDouble(content, i);

        conical_surfaces.emplace(temp.index, temp);
    }
    else if(keyword == "SPHERICAL_SURFACE") {
        SPHERICAL_SURFACE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);
        //radius半径
        temp.radius = getDouble(content, i);

        spherical_surfaces.emplace(temp.index, temp);
    }
    else if(keyword == "TOROIDAL_SURFACE") {
        TOROIDAL_SURFACE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);
        //major_radius大半径
        temp.major_radius = getDouble(content, i);
        //minor_radius小半径
        temp.minor_radius = getDouble(content, i);

        toroidal_surfaces.emplace(temp.index, temp);
    }
    else if(keyword == "AXIS2_PLACEMENT_3D") {
        AXIS2_PLACEMENT_3D temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //point笛卡尔点
        temp.point = getAnIndex(content, i);
        //dierctionZ轴、X轴
        temp.directionZ = getAnIndex(content, i);
        temp.directionX = getAnIndex(content, i);

        axis2_pacement_3ds.emplace(temp.index, temp);
    }
    else if(keyword == "VERTEX_LOOP") {
        VERTEX_LOOP temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //vertex顶点
        temp.vertex = getAnIndex(content, i);

        vertex_loops.emplace(temp.index, temp);
    }
    else if(keyword == "EDGE_LOOP") {
        EDGE_LOOP temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //edges有向边
        getIndexGroup(temp.edges, content, i);

        edge_loops.emplace(temp.index, temp);
    }
    else if(keyword == "ORIENTED_EDGE") {
        ORIENTED_EDGE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //curve无向边
        temp.curve = getAnIndex(content, i);
        //flag = T/F
        temp.flag = getFlag(content, i);

        oriented_edges.emplace(temp.index, temp);
    }
    else if(keyword == "EDGE_CURVE") {
        EDGE_CURVE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //vertex1顶点1
        temp.vertex1 = getAnIndex(content, i);
        //vertex2顶点2
        temp.vertex2 = getAnIndex(content, i);
        //edge边类（LINE、CIRCLE等）
        temp.edge = getAnIndex(content, i);
        //flag = T/F
        temp.flag = getFlag(content, i);

        edge_curves.emplace(temp.index, temp);
    }
    else if(keyword == "VERTEX_POINT") {
        VERTEX_POINT temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //point笛卡尔点
        temp.point = getAnIndex(content, i);

        vertex_points.emplace(temp.index, temp);
    }
    else if(keyword == "CARTESIAN_POINT") {
        CARTESIAN_POINT temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //(x,y,z)笛卡尔坐标
        temp.x = getDouble(content, i);
        temp.y = getDouble(content, i);
        temp.z = getDouble(content, i);

        cartesian_points.emplace(temp.index, temp);
    }
    else if(keyword == "LINE") {
        LINE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //point笛卡尔点
        temp.point = getAnIndex(content, i);
        //vector向量
        temp.vector = getAnIndex(content, i);

        lines.emplace(temp.index, temp);
    }
    else if(keyword == "VECTOR") {
        VECTOR temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //direction方向
        temp.direction = getAnIndex(content, i);
        //n长度？
        temp.n = getDouble(content, i);

        vectors.emplace(temp.index, temp);
    }
    else if(keyword == "DIRECTION") {
        DIRECTION temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //(x,y,z)笛卡尔向量坐标
        temp.x = getDouble(content, i);
        temp.y = getDouble(content, i);
        temp.z = getDouble(content, i);

        directions.emplace(temp.index, temp);
    }
    else if(keyword == "CIRCLE") {
        CIRCLE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);
        //radius半径
        temp.radius = getDouble(content, i);

        circles.emplace(temp.index, temp);
    }
}

bool readSTPtoCPP(string filepath)
{
    fstream f;
    f.open(filepath, ios::in);
    if(!f.is_open())
    {
        cout << "open failed!" << endl;
        return false;
    }
    string index, keyword, content; //索引，关键词，内容
    char ch;
    while(EOF != (ch = f.get())) {
        if(ch == '#') {
            index = readIndex(f);
            keyword = readKeyword(f);
            if(isKeyword(keyword)) {  //判断是有效关键词就进行映射
                content = readContent(f);
                instantiation(index, keyword, content);
            }
            else {
                ignoreContent(f); //无效关键词忽略语句内容
            }
        }
    }
    f.close();
    return true;
}

void makeIndexTable()
{
    for(auto it = closed_shells.begin(); it != closed_shells.end(); it++)
    {
        for(auto itt = it->second.faces.begin(); itt != it->second.faces.end(); itt++)
        {
            advanced_faces.find(*itt)->second.upIndex = it->second.index;
        }
    }
    for(auto it = advanced_faces.begin(); it != advanced_faces.end(); it++)
    {
        for(auto itt = it->second.bounds.begin(); itt != it->second.bounds.end(); itt++)
        {
            string type = it->second.indexType(*itt);
            if(type == "FACE_OUTER_BOUND") {
                face_outer_bounds.find(*itt)->second.upIndex = it->second.index;
            }
            else if(type == "FACE_BOUND") {
                face_bounds.find(*itt)->second.upIndex = it->second.index;
            }
        }

        string type = it->second.indexType(it->second.face);
        it->second.faceType = type;
        if(type == "PLANE") {
            planes.find(it->second.face)->second.upIndex = it->second.index;
        }
        else if(type == "CYLINDRICAL_SURFACE") {
            cylindrical_surfaces.find(it->second.face)->second.upIndex = it->second.index;
        }
        else if(type == "CONICAL_SURFACE") {
            conical_surfaces.find(it->second.face)->second.upIndex = it->second.index;
        }
        else if(type == "SPHERICAL_SURFACE") {
            spherical_surfaces.find(it->second.face)->second.upIndex = it->second.index;
        }
        else if(type == "TOROIDAL_SURFACE") {
            toroidal_surfaces.find(it->second.face)->second.upIndex = it->second.index;
        }
    }
    for(auto it = face_outer_bounds.begin(); it != face_outer_bounds.end(); it++)
    {
        string type = it->second.indexType(it->second.loop);
        if(type == "VERTEX_LOOP") {
            vertex_loops.find(it->second.loop)->second.upIndex = it->second.index;
        }
        else if(type == "EDGE_LOOP") {
            edge_loops.find(it->second.loop)->second.upIndex = it->second.index;
        }
    }
    for(auto it = face_bounds.begin(); it != face_bounds.end(); it++)
    {
        string type = it->second.indexType(it->second.loop);
        if(type == "VERTEX_LOOP") {
            vertex_loops.find(it->second.loop)->second.upIndex = it->second.index;
        }
        else if(type == "EDGE_LOOP") {
            edge_loops.find(it->second.loop)->second.upIndex = it->second.index;
        }
    }
    for(auto it = planes.begin(); it != planes.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
    for(auto it = cylindrical_surfaces.begin(); it != cylindrical_surfaces.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
    for(auto it = conical_surfaces.begin(); it != conical_surfaces.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
    for(auto it = spherical_surfaces.begin(); it != spherical_surfaces.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
    for(auto it = toroidal_surfaces.begin(); it != toroidal_surfaces.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
    for(auto it = axis2_pacement_3ds.begin(); it != axis2_pacement_3ds.end(); it++)
    {
        cartesian_points.find(it->second.point)->second.upIndex = it->second.index;

        directions.find(it->second.directionZ)->second.upIndex = it->second.index;
        directions.find(it->second.directionX)->second.upIndex = it->second.index;
    }
    for(auto it = vertex_loops.begin(); it != vertex_loops.end(); it++)
    {
        vertex_points.find(it->second.vertex)->second.upIndexes.emplace_back(it->second.index);
    }
    for(auto it = edge_loops.begin(); it != edge_loops.end(); it++)
    {
        for(auto itt = it->second.edges.begin(); itt != it->second.edges.end(); itt++)
        {
            oriented_edges.find(*itt)->second.upIndex = it->second.index;
        }
    }
    for(auto it = oriented_edges.begin(); it != oriented_edges.end(); it++)
    {
        edge_curves.find(it->second.curve)->second.upIndexes.emplace_back(it->second.index);
    }
    for(auto it = edge_curves.begin(); it != edge_curves.end(); it++)
    {
        vertex_points.find(it->second.vertex1)->second.upIndexes.emplace_back(it->second.index);
        vertex_points.find(it->second.vertex2)->second.upIndexes.emplace_back(it->second.index);

        string type = it->second.indexType(it->second.edge);
        it->second.edgeType = type;
        if(type == "LINE") {
            lines.find(it->second.edge)->second.upIndex = it->second.index;
        }
        else if(type == "CIRCLE") {
            circles.find(it->second.edge)->second.upIndex = it->second.index;
        }
    }
    for(auto it = vertex_points.begin(); it != vertex_points.end(); it++)
    {
        cartesian_points.find(it->second.point)->second.upIndex = it->second.index;
    }
    for(auto it = lines.begin(); it != lines.end(); it++)
    {
        cartesian_points.find(it->second.point)->second.upIndex = it->second.index;
        vectors.find(it->second.vector)->second.upIndex = it->second.index;
    }
    for(auto it = vectors.begin(); it != vectors.end(); it++)
    {
        directions.find(it->second.direction)->second.upIndex = it->second.index;
    }
    for(auto it = circles.begin(); it != circles.end(); it++)
    {
        axis2_pacement_3ds.find(it->second.axis2)->second.upIndex = it->second.index;
    }
}

string readSTP(string filepath)
{
    fstream f;
    f.open(filepath, ios::in);
    //文件打开方式选项：
    //　ios::in　　　　= 0x01,　//供读，文件不存在则创建(ifstream默认的打开方式)
    //　ios::out　　　 = 0x02,　//供写，文件不存在则创建，若文件已存在则清空原内容(ofstream默认的打开方式)
    //　ios::ate　　　 = 0x04,　//文件打开时，指针在文件最后。可改变指针的位置，常和in、out联合使用
    //　ios::app　　　 = 0x08,　//供写，文件不存在则创建，若文件已存在则在原文件内容后写入新的内容，指针位置总在最后
    //　ios::trunc　　 = 0x10,　//在读写前先将文件长度截断为0（默认）
    //　ios::nocreate　= 0x20,　//文件不存在时产生错误，常和in或app联合使用
    //　ios::noreplace = 0x40,　//文件存在时产生错误，常和out联合使用
    //　ios::binary　　= 0x80　 //二进制格式文件
    if(!f)
    {
        cout << "open failed!打开文件失败！" << endl;
        return NULL;
    }
    //按字节读入
    string s;
    char ch;
    while(EOF != (ch = f.get())){
        s += ch;
    }
    f.close();
    return s;
}

string findIndexType(int index)
{
    if(closed_shells.find(index) != closed_shells.end()) {
        return "CLOSED_SHELL";
    }
    else if(advanced_faces.find(index) != advanced_faces.end()) {
        return "ADVANCED_FACE";
    }
    else if(face_outer_bounds.find(index) != face_outer_bounds.end()) {
        return "FACE_OUTER_BOUND";
    }
    else if(face_bounds.find(index) != face_bounds.end()) {
        return "FACE_BOUND";
    }
    else if(planes.find(index) != planes.end()) {
        return "PLANE";
    }
    else if(cylindrical_surfaces.find(index) != cylindrical_surfaces.end()) {
        return "CYLINDRICAL_SURFACE";
    }
    else if(conical_surfaces.find(index) != conical_surfaces.end()) {
        return "CONICAL_SURFACE";
    }
    else if(spherical_surfaces.find(index) != spherical_surfaces.end()) {
        return "SPHERICAL_SURFACE";
    }
    else if(toroidal_surfaces.find(index) != toroidal_surfaces.end()) {
        return "TOROIDAL_SURFACE";
    }
    else if(axis2_pacement_3ds.find(index) != axis2_pacement_3ds.end()) {
        return "AXIS2_PLACEMENT_3D";
    }
    else if(vertex_loops.find(index) != vertex_loops.end()) {
        return "VERTEX_LOOP";
    }
    else if(edge_loops.find(index) != edge_loops.end()) {
        return "EDGE_LOOP";
    }
    else if(oriented_edges.find(index) != oriented_edges.end()) {
        return "ORIENTED_EDGE";
    }
    else if(edge_curves.find(index) != edge_curves.end()) {
        return "EDGE_CURVE";
    }
    else if(vertex_points.find(index) != vertex_points.end()) {
        return "VERTEX_POINT";
    }
    else if(cartesian_points.find(index) != cartesian_points.end()) {
        return "CARTESIAN_POINT";
    }
    else if(lines.find(index) != lines.end()) {
        return "LINE";
    }
    else if(vectors.find(index) != vectors.end()) {
        return "VECTOR";
    }
    else if(directions.find(index) != directions.end()) {
        return "DIRECTION";
    }
    else if(circles.find(index) != circles.end()) {
        return "CIRCLE";
    }
    return "";
}

void output()
{
    //STEP文件映射
    cout << endl << "STEP file:" << endl << endl;
    cout << "CLOSED_SHELL" << endl;
    for(auto i = closed_shells.begin(); i != closed_shells.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " faces=";
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
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "FACE_OUTER_BOUND" << endl;
    for(auto i = face_outer_bounds.begin(); i != face_outer_bounds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " loop=" << i->second.loop << " flag=" << i->second.flag;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "FACE_BOUND" << endl;
    for(auto i = face_bounds.begin(); i != face_bounds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " loop=" << i->second.loop << " flag=" << i->second.flag;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "PLANE" << endl;
    for(auto i = planes.begin(); i != planes.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "CYLINDRICAL_SURFACE" << endl;
    for(auto i = cylindrical_surfaces.begin(); i != cylindrical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "CONICAL_SURFACE" << endl;
    for(auto i = conical_surfaces.begin(); i != conical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius << " coneAngle=" << i->second.coneAngle;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "SPHERICAL_SURFACE" << endl;
    for(auto i = spherical_surfaces.begin(); i != spherical_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "TOROIDAL_SURFACE" << endl;
    for(auto i = toroidal_surfaces.begin(); i != toroidal_surfaces.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " major_radius=" << i->second.major_radius << " minor_radius=" << i->second.minor_radius;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "AXIS2_PLACEMENT_3D" << endl;
    for(auto i = axis2_pacement_3ds.begin(); i != axis2_pacement_3ds.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point << " directionZ=" << i->second.directionZ << " directionX=" << i->second.directionX;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "VERTEX_LOOP" << endl;
    for(auto i = vertex_loops.begin(); i != vertex_loops.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " vertex=" << i->second.vertex;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "EDGE_LOOP" << endl;
    for(auto i = edge_loops.begin(); i != edge_loops.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " edges=";
        for(auto j = i->second.edges.begin(); j != i->second.edges.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "ORIENTED_EDGE" << endl;
    for(auto i = oriented_edges.begin(); i != oriented_edges.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " curve=" << i->second.curve << " flag=" << i->second.flag;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "EDGE_CURVE" << endl;
    for(auto i = edge_curves.begin(); i != edge_curves.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " vertex1=" << i->second.vertex1 << " vertex2=" << i->second.vertex2 << " edge=" << i->second.edge << " flag=" << i->second.flag;
        cout << " upIndex=";
        for(auto j = i->second.upIndexes.begin(); j != i->second.upIndexes.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << endl;
    }
    cout << "VERTEX_POINT" << endl;
    for(auto i = vertex_points.begin(); i != vertex_points.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point;
        cout << " upIndex=";
        for(auto j = i->second.upIndexes.begin(); j != i->second.upIndexes.end(); j++)
        {
            cout << *j << ' ';
        }
        cout << endl;
    }
    cout << "CARTESIAN_POINT" << endl;
    for(auto i = cartesian_points.begin(); i != cartesian_points.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " (x,y,z)=" << i->second.x << ", " << i->second.y << ", " << i->second.z;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "LINE" << endl;
    for(auto i = lines.begin(); i != lines.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " point=" << i->second.point << " vector=" << i->second.vector;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "VECTOR" << endl;
    for(auto i = vectors.begin(); i != vectors.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " direction=" << i->second.direction << " n=" << i->second.n;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "DIRECTION" << endl;
    for(auto i = directions.begin(); i != directions.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " (x,y,z)=" << i->second.x << ", " << i->second.y << ", " << i->second.z;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }
    cout << "CIRCLE" << endl;
    for(auto i = circles.begin(); i != circles.end(); i++)
    {
        cout << '#' << i->first << " text=" << i->second.text << " axis2=" << i->second.axis2 << " radius=" << i->second.radius;
        cout << " upIndex=" << i->second.upIndex;
        cout << endl;
    }

    //属性邻接图AAG
    cout << endl << endl << "AAG:" << endl << endl;
    for(auto ad_it = advanced_faces.begin(); ad_it != advanced_faces.end(); ad_it++)
    {
        for(auto adj_it = ad_it->second.adjacentFaces.begin(); adj_it != ad_it->second.adjacentFaces.end(); adj_it++)
        {
            CommonEdge commonEdge = adj_it->second;
            cout << ad_it->second.faceType << ad_it->second.index << '/' << advanced_faces.find(adj_it->first)->second.faceType << advanced_faces.find(adj_it->first)->second.index << ' '; 
            cout << "COMMONEDGE." << "index = " << commonEdge.index << ", edgeType = " << commonEdge.edgeType << ", concavity = " << commonEdge.concavity << ", angle = " << commonEdge.angle << endl;
        }
    }
}