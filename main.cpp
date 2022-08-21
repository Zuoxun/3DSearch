#include "LexAnalysis.h"
#include "LexAnalysis.cpp"

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
        }
        else cout << "ReadSTPtoCPP failed!" << endl;
    }
    return 0;
}