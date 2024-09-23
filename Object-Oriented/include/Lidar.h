#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

#include "Subject.h"

using namespace std;


class Lidar : public Subject
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

    nlohmann::json save()
    {
        nlohmann::json lidarJson;
        lidarJson["激光雷达型号"] = model;
        lidarJson["通道数"] = channels;
        lidarJson["测试范围"] = range;
        lidarJson["功耗"] = power;
        
        return lidarJson;
    }
};


#endif // LIDAR_H