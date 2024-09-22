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
    nlohmann::json j;
    ifs >> j;
    for (int i = 0; i < 10; i++)
    {   
        string _id = "车辆" + to_string(i+1);
        Car car;
        car.id = j[_id]["车辆ID"];
        car.student_id = j[_id]["学号"];
        car.student_name = j[_id]["姓名"];
        car.chassis.model = j[_id]["底盘信息"]["底盘型号"];
        car.chassis.wheelbase = j[_id]["底盘信息"]["轴距"];
        car.chassis.track = j[_id]["底盘信息"]["轮距"];
        car.chassis.min_ground_clearance = j[_id]["底盘信息"]["最小离地间隙"];
        car.chassis.min_turning_radius = j[_id]["底盘信息"]["最小转弯半径"];
        car.chassis.drive_type = j[_id]["底盘信息"]["驱动形式"];
        car.chassis.max_range = j[_id]["底盘信息"]["最大行程"];
        car.chassis.tire_model = j[_id]["底盘信息"]["轮胎型号"];
        car.chassis.tire_size = j[_id]["底盘信息"]["轮胎尺寸"];
        car.agx_kit.ai = j[_id]["AGX套件信息"]["AI性能"];
        car.agx_kit.model = j[_id]["AGX套件信息"]["AGXKit型号"];
        car.agx_kit.cuda_cores = j[_id]["AGX套件信息"]["CUDA核心数"];  // 修改键名
        car.agx_kit.tensor_cores = j[_id]["AGX套件信息"]["Tensor核心数"];  // 修改键名
        car.agx_kit.memory = j[_id]["AGX套件信息"]["内存大小"];  // 修改键名
        car.agx_kit.storage = j[_id]["AGX套件信息"]["存储大小"];
        car.stereo_camera.camera = j[_id]["立体相机信息"]["相机型号"];  // 修改键名
        car.stereo_camera.model = j[_id]["立体相机信息"]["立体相机型号"];  // 修改键名
        car.lidar.channels = j[_id]["激光雷达信息"]["通道数"];
        car.lidar.model = j[_id]["激光雷达信息"]["激光雷达型号"];  // 修改键名
        car.lidar.power = j[_id]["激光雷达信息"]["功耗"];
        car.lidar.range = j[_id]["激光雷达信息"]["测试范围"];
        car.gyroscope.manufacturer = j[_id]["陀螺仪信息"]["厂家"];
        car.gyroscope.model = j[_id]["陀螺仪信息"]["陀螺仪型号"];  // 修改键名
        car.lcd.model = j[_id]["液晶显示屏信息"]["液晶显示屏型号"];  // 修改键名
        car.lcd.size = j[_id]["液晶显示屏信息"]["液晶显示屏尺寸"];  // 修改键名
        car.battery.charge_time = j[_id]["电池模块信息"]["充电时长"];
        car.battery.output = j[_id]["电池模块信息"]["对外供电"];
        car.battery.parameter = j[_id]["电池模块信息"]["电池参数"];
        cars.push_back(car);
    }
    cout << "*************" << endl;
    cout << ">>读取成功!<<" << endl;
    cout << "*************" << endl;
    cout << endl;

    return cars;
}





#endif // UTILS_H