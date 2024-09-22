#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;


class Lidar
{
public:
    string model = "RS-Helios-16p";
    string channels = "16";
    string range = "100";
    string power = "8";

    void set()
    {
        cout << "请输入激光雷达型号: ";
        cin >> model;
        cout << "请输入通道数: ";
        cin >> channels;
        cout << "请输入测试范围: ";
        cin >> range;
        cout << "请输入功耗: ";
        cin >> power;
    }

    void print()
    {
        cout << setw(20) << "激光雷达型号: " << model << endl;
        cout << setw(20) << "通道数: " << channels << endl;
        cout << setw(20) << "测试范围: " << range << endl;
        cout << setw(20) << "功耗: " << power << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "激光雷达信息:" << endl;
        ofs << "        (a) 型号: " << model << endl;
        ofs << "        (b) 通道数: " << channels << endl;
        ofs << "        (c) 测试范围: " << range << endl;
        ofs << "        (d) 功耗: " << power << endl;
    }
};


#endif // LIDAR_H