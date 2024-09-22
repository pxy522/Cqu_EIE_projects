/**
 * @file utils.h
 * @author pxy522 (pxy174@gamil.com)
 * @brief 程序所需的工具函数
 * @version 0.1
 * @date 2024-09-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UTILS_H
#define UTILS_H

#include "Cars.h"

using namespace std;

/**
 * @brief 修改车辆信息
 * 
 */
void modifyCarInfo( vector<Car>& cars )
{
    cout << "请输入要修改的车辆ID: ";
    string id;
    cin >> id;
    for (int i = 0; i < cars.size(); i++)
    {   
        if (cars[i].id == id)
        {
            cars[i].set();
            return;
        }
    }
    cout << "未找到该车辆信息!" << endl;
}

/**
 * @brief 从json文件中读取车辆信息 
 * 
 * @return vector<Car>
 */
vector<Car> readJsonInfo( string path )
{
    vector<Car> cars;
    ifstream ifs(path);
    if (!ifs.is_open())
    {
        cout << "文件打开失败!" << endl;
        return cars;
    }
    json carsJson;
    ifs >> carsJson;
    for (int i = 0; i < carsJson.size(); i++)
    {
        Car car;
        car.id = carsJson[i]["车辆ID"];
        car.student_id = carsJson[i]["学号"];
        car.student_name = carsJson[i]["姓名"];

        cars.push_back(car);
    }
    ifs.close();
    return cars;
}





#endif // UTILS_H