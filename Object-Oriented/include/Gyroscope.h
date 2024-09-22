#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

class Gyroscope
{
public:
    string model = "CH110";
    string manufacturer = "NXP";

    void set()
    {
        cout << "请输入陀螺仪型号: ";
        cin >> model;
        cout << "请输入厂家: ";
        cin >> manufacturer;
    }

    void print()
    {
        cout << setw(20) << "陀螺仪型号: " << model << endl;
        cout << setw(20) << "厂家: " << manufacturer << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "9轴陀螺仪信息:" << endl;
        ofs << "        (a) 型号: " << model << endl;
        ofs << "        (b) 厂家: " << manufacturer << endl;
    }
};


#endif // GYROSCOPE_H