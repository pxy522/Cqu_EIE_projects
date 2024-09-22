#ifndef BATTERY_H
#define BATTERY_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;


class Battery
{
public:
    string parameter = "24V/15Ah";
    string output = "24V";
    string charge_time = "2";

    void set()
    {
        cout << "请输入电池参数: ";
        cin >> parameter;
        cout << "请输入对外供电: ";
        cin >> output;
        cout << "请输入充电时长: ";
        cin >> charge_time;
    }

    void print()
    {
        cout << setw(20) << "电池参数: " << parameter << endl;
        cout << setw(20) << "对外供电: " << output << endl;
        cout << setw(20) << "充电时长: " << charge_time << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "电池模块信息:" << endl;
        ofs << "        (a) 参数: " << parameter << endl;
        ofs << "        (b) 对外供电: " << output << endl;
        ofs << "        (c) 充电时长: " << charge_time << endl;
    }
};


#endif // BATTERY_H
