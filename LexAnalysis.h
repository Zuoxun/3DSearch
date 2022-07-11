#pragma once
#pragma execution_character_set("utf-8") //解决中文编码
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <cstring>
#include "StpDataStructure.h"

using namespace std;

//从stp读句首索引号
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

//从stp读关键词
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

//从stp读语句内容
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

//略过stp无效关键词的语句内容
void ignoreContent(fstream &f)
{
    char ch;
    while(';' != (ch = f.get())) {} //省略到语句结束的';'字符
}

//判断是不是有效的keyword
bool isKeyword(string keyword)
{
    if(keywords.count(keyword) > 0) return true;
    else return false;
}

//将一个用科学计数法表示的浮点数string转换为double
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

//判断一个字符是不是数字
bool isNumber(char ch)
{
    if(ch >= '0' && ch <= '9') return true;
    else return false;
}

//获得content里的text文本（从第i位开始遍历）
string getText(string content, int &i)
{
    string s;
    while(content[i++] != '\'') {} //定位到第一个'字符后
    while(content[i] != '\'') {
        s += content[i++];
    }
    return s;
}

//获得content里的下一组索引（从第i位开始遍历）
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
            indexGroup.push_back(atoi(s.c_str()));
        }
        else i++;
    }
}

//获得content里的下一个索引（从第i位开始遍历）
int getAnIndex(string content, int &i)
{
    string s;
    while(content[i++] != '#') {}
    while(isNumber(content[i])) {
        s += content[i++];
    }
    return atoi(s.c_str());
}

//获得content里的下一个浮点数double（从第i位开始遍历）
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

//获得content里的下一个flag:T/F（从第i位开始遍历）
bool getFlag(string content, int &i)
{
    while(content[i] != 'T' && content[i] != 'F') {i++;}
    return (content[i] == 'T') ? true : false;
}

//将一个stp语句提取出的索引、关键词、内容进行实例化
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
    if(keyword == "ADVANCED_FACE") {
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
    if(keyword == "FACE_OUTER_BOUND") {
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
    if(keyword == "FACE_BOUND") {
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
    if(keyword == "PLANE") {
        PLANE temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //axis2轴二定位
        temp.axis2 = getAnIndex(content, i);

        planes.emplace(temp.index, temp);
    }
    if(keyword == "CYLINDRICAL_SURFACE") {
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
    if(keyword == "CONICAL_SURFACE") {
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
    if(keyword == "SPHERICAL_SURFACE") {
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
    if(keyword == "TOROIDAL_SURFACE") {
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
    if(keyword == "AXIS2_PLACEMENT_3D") {
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
    if(keyword == "EDGE_LOOP") {
        EDGE_LOOP temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //edges有向边
        getIndexGroup(temp.edges, content, i);

        edge_loops.emplace(temp.index, temp);
    }
    if(keyword == "ORIENTED_EDGE") {
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
    if(keyword == "EDGE_CURVE") {
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
    if(keyword == "VERTEX_POINT") {
        VERTEX_POINT temp;
        temp.index = atoi(index.c_str());
        //text备注
        int i = 0;
        temp.text = getText(content, i);
        //point笛卡尔点
        temp.point = getAnIndex(content, i);

        vertex_points.emplace(temp.index, temp);
    }
    if(keyword == "CARTESIAN_POINT") {
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
    if(keyword == "LINE") {
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
    if(keyword == "VECTOR") {
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
    if(keyword == "DIRECTION") {
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
    if(keyword == "CIRCLE") {
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

//读入stp文件并映射至数据结构中
bool readSTPtoCPP(string filepath)
{
    fstream f;
    f.open(filepath, ios::in);
    if(!f)
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
    return true;
}

//读取stp文件测试函数
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

//查询索引号属于什么类
string findIndexType(int index)
{
    if(closed_shells.count(index) > 0) {
        return "closed_shells";
    }
    if(advanced_faces.count(index) > 0) {
        return "advanced_faces";
    }
    if(face_outer_bounds.count(index) > 0) {
        return "face_outer_bounds";
    }
    if(face_bounds.count(index) > 0) {
        return "face_bounds";
    }
    if(planes.count(index) > 0) {
        return "planes";
    }
    if(cylindrical_surfaces.count(index) > 0) {
        return "cylindrical_surfaces";
    }
    if(conical_surfaces.count(index) > 0) {
        return "conical_surfaces";
    }
    if(spherical_surfaces.count(index) > 0) {
        return "spherical_surfaces";
    }
    if(toroidal_surfaces.count(index) > 0) {
        return "toroidal_surfaces";
    }
    if(axis2_pacement_3ds.count(index) > 0) {
        return "axis2_pacement_3ds";
    }
    if(edge_loops.count(index) > 0) {
        return "edge_loops";
    }
    if(oriented_edges.count(index) > 0) {
        return "oriented_edges";
    }
    if(edge_curves.count(index) > 0) {
        return "edge_curves";
    }
    if(vertex_points.count(index) > 0) {
        return "vertex_points";
    }
    if(cartesian_points.count(index) > 0) {
        return "cartesian_points";
    }
    if(lines.count(index) > 0) {
        return "lines";
    }
    if(vectors.count(index) > 0) {
        return "vectors";
    }
    if(directions.count(index) > 0) {
        return "directions";
    }
}
