#include <iostream>
#include <string>
#include <cstring>
#include <math.h>
#include <map>
using namespace std;

struct A
{
public:
    int aa = 2022;
    virtual void func()
    {
        cout << "A" << endl;
    }
};

struct B : public A
{
public:
    B(int temp):aa(temp),bb(temp+1)
    {
    }
    int aa = 2021;
    int bb;
    void func(int t)
    {
        cout << "B+" << t << endl;
    }
    void func2()
    {
        cout<<"aa="<<aa<<endl;
    }
};

struct C : public A
{
public:
    
};

map<int,A> m;