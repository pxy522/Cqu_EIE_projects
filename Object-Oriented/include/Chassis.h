#ifndef CHASSIS_H
#define CHASSIS_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

class Chassis
{
public:
    string id;
    string model = "SCOUT MINI";
    string wheelbase = "451";
    string track = "490";
    string min_ground_clearance = "115";
    string min_turning_radius = "0";
    string drive_type = "四轮四驱";
    string max_range = "10";
    string tire_model = "公路轮、麦克纳姆轮";
    string tire_size = "175";

    void set()
    {
        cout << "请输入底盘编号: ";
        cin >> id;
        cout << "请输入底盘型号: ";
        cin >> model;
        cout << "请输入轴距: ";
        cin >> wheelbase;
        cout << "请输入轮距: ";
        cin >> track;
        cout << "请输入最小离地间隙: ";
        cin >> min_ground_clearance;
        cout << "请输入最小转弯半径: ";
        cin >> min_turning_radius;
        cout << "请输入驱动形式: ";
        cin >> drive_type;
        cout << "请输入最大行程: ";
        cin >> max_range;
        cout << "请输入轮胎型号: ";
        cin >> tire_model;
        cout << "请输入轮胎尺寸: ";
        cin >> tire_size;
    }

    void print()
    {
        cout << setw(20) << "底盘编号: " << id << endl;
        cout << setw(20) << "底盘型号: " << model << endl;
        cout << setw(20) << "轴距: " << wheelbase << endl;
        cout << setw(20) << "轮距: " << track << endl;
        cout << setw(20) << "最小离地间隙: " << min_ground_clearance << endl;
        cout << setw(20) << "最小转弯半径: " << min_turning_radius << endl;
        cout << setw(20) << "驱动形式: " << drive_type << endl;
        cout << setw(20) << "最大行程: " << max_range << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "底盘信息:" << endl;
        ofs << "        (a) 底盘编号: " << id << endl;
        ofs << "        (b) 底盘型号: " << model << endl;
        ofs << "        (c) 轴距: " << wheelbase << endl;
        ofs << "        (d) 轮距: " << track << endl;
        ofs << "        (e) 最小离地间隙: " << min_ground_clearance << endl;
        ofs << "        (f) 最小转弯半径: " << min_turning_radius << endl;
        ofs << "        (g) 驱动形式: " << drive_type << endl;
        ofs << "        (h) 最大行程: " << max_range << endl;
    }


};





#endif // CHASSIS_H