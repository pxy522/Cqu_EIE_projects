/**
 * @file Cars.h
 * @author pxy522 (pxy174@gamil.com)
 * @brief 按照标准规定来说，类的定义应该放在头文件中，而类的实现应该放在源文件中。这样做的好处是，头文件中只有类的声明，不包含实现，这样可以减少编译时间，提高编译效率。
 *        但是在实际开发中，如果类的实现比较简单，可以直接将类的定义和实现放在头文件中，这样可以减少文件数量，方便管理。
 * @version 0.1
 * @date 2024-09-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CARS_H
#define CARS_H

#include <vector>
#include <chrono>
#include <sstream>

/* 各属性板块 */
#include "Chassis.h"
#include "AGXKit.h"
#include "StereoCamera.h"
#include "Lidar.h"
#include "Gyroscope.h"
#include "LCD.h"
#include "Battery.h"

using namespace std;

class Car
{
public:
    string id;
    Chassis chassis;
    AGXKit agx_kit;
    StereoCamera stereo_camera;
    Lidar lidar;
    Gyroscope gyroscope;
    LCD lcd;
    Battery battery;
    string student_id;
    string student_name;

    void inputCarInfo()
    {
        cout << "请输入学号: ";
        cin >> student_id;
        cout << "请输入姓名: ";
        cin >> student_name;
        cout << "生成车辆ID..." << endl;
        id = "cqusn" + getCurrentTime() + student_id;       // 生成车辆ID
        chassis.id = "dp" + student_id;                     // 生成底盘编号
    }

    void print()
    {
        cout << "--------------------------------" << endl;
        cout << "车辆ID: " << id << endl;
        cout << "底盘编号: " << chassis.id << endl;
        cout << "学号: " << student_id << " 姓名: " << student_name << endl;
        cout << "--------------------------------" << endl;
    }

    string getCurrentTime()
    {
        auto now = chrono::system_clock::now();
        time_t now_time = chrono::system_clock::to_time_t(now);
        tm ltm;
        localtime_s(&ltm, &now_time);

        ostringstream ss;
        ss << ltm.tm_year + 1900
            << setw(2) << setfill('0') << ltm.tm_mon + 1
            << setw(2) << setfill('0') << ltm.tm_mday;

        string time = ss.str();
        cout << time << endl;
        return time;
    }

    void save(ofstream &ofs)
    {
        ofs << "车辆信息:" << endl;
        ofs << "        (a) 车辆ID: " << id << endl;
        ofs << "        (b) 底盘编号: " << chassis.id << endl;
        ofs << "        (c) 学号: " << student_id << endl;
        ofs << "        (d) 姓名: " << student_name << endl;
        chassis.save(ofs);
        agx_kit.save(ofs);
        stereo_camera.save(ofs);
        lidar.save(ofs);
        gyroscope.save(ofs);
        lcd.save(ofs);
        battery.save(ofs);
    }
};






#endif // CARS_H