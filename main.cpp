#include "LexAnalysis.h"
#include "LexAnalysis.cpp"
#include "FeatureAnalysis.h"
#include "FeatureAnalysis.cpp"

int main()
{
    string filepath;
    while(getline(cin, filepath))
    {
        clearData();
        if(readSTPtoCPP(filepath)) {
            cout << "ReadSTPtoCPP succeeded!" << endl;
            makeIndexTable();
            output();
            cylinder_and_circularHole();
        }
        else cout << "ReadSTPtoCPP failed!" << endl;
    }
    return 0;
}