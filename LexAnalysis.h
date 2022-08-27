#pragma once

#include <fstream>
#include <stdlib.h>
#include "StpDataStructure.h"
#include "StpDataStructure.cpp"

//清空数据结构容器
void clearData();

//从stp读句首索引号
string readIndex(fstream &f);

//从stp读关键词
string readKeyword(fstream &f);

//从stp读语句内容
string readContent(fstream &f);

//略过stp无效关键词的语句内容
void ignoreContent(fstream &f);

//判断是不是有效的keyword
bool isKeyword(string keyword);

//将一个用科学计数法表示的浮点数string转换为double
double toDouble(string str);

//判断一个字符是不是数字
bool isNumber(char ch);

//获得content里的text文本（从第i位开始遍历）
string getText(string content, int &i);

//获得content里的下一组索引（从第i位开始遍历）
void getIndexGroup(vector<int> &indexGroup, string content, int &i);

//获得content里的下一个索引（从第i位开始遍历）
int getAnIndex(string content, int &i);

//获得content里的下一个浮点数double（从第i位开始遍历）
double getDouble(string content, int &i);

//获得content里的下一个flag:T/F（从第i位开始遍历）
bool getFlag(string content, int &i);

//将一个stp语句提取出的索引、关键词、内容进行实例化
void instantiation(string index, string keyword, string content);

//读入stp文件并映射至数据结构中
bool readSTPtoCPP(string filepath);

//梳理索引表
void makeIndexTable();

//读取stp文件测试函数
string readSTP(string filepath);

//查询索引号属于什么类
string findIndexType(int index);

//输出映射结果
void output();