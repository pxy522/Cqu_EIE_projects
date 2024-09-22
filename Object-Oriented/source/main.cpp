/**
 * @file main.cpp
 * @author pxy522 (pxy174@gamil.com)
 * @brief 智能小车信息管理主程序
 * @version 0.1
 * @date 2024-09-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Cars.h"
#include "utils.h"

using namespace std;


int main()
{
    vector<Car> cars;
    /* 循环 */
    while ( true )
    {
        cout << "--------------------------------" << endl;
        cout << "1.录入车辆信息" << endl;
        cout << "2.修改车辆信息" << endl;
        cout << "3.保存车辆信息" << endl;
        cout << "4.从文件中读取车辆信息" << endl;
        cout << "5.退出" << endl;
        cout << "--------------------------------" << endl;
        cout << "请选择操作: ";
        int choice;
        cin >> choice;
        switch (choice)
        {
        case 1:
            {
                if (cars.size() >= 10)
                {
                    cout << "车辆信息已满!" << endl;
                    break;
                }
                for (int i = 0; i < 10; i++)
                {
                    Car car;
                    car.inputCarInfo();
                    cars.push_back(car);
                }
                break;
            }
        case 2:
            modifyCarInfo(cars);
            break;
        case 3:
            {
                string path;
                cout << "请输入保存路径: ";
                cin >> path;
                ofstream ofs(path);
                for (int i = 0; i < cars.size(); i++)
                {
                    cars[i].save(ofs);
                }
                ofs.close();
                break;
            }
        case 4:
            {
                string path;
                cout << "请输入文件路径: ";
                cin >> path;
                cars = readJsonInfo(path);
                break;
            }
        case 5:
            return 0;
        default:
            break;
        }
    }
}