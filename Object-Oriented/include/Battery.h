#ifndef BATTERY_H
#define BATTERY_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

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
        json batteryJson;
        batteryJson["电池参数"] = parameter;
        batteryJson["对外供电"] = output;
        batteryJson["充电时长"] = charge_time;
        ofs << batteryJson.dump(4) << endl;
    }
};


#endif // BATTERY_H
