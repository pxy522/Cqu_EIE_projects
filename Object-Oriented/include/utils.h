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
        car.chassis.id = carsJson[i]["底盘编号"];
        car.student_id = carsJson[i]["学号"];
        car.student_name = carsJson[i]["姓名"];
        
        car.chassis.model = carsJson[i]["底盘型号"];
        car.chassis.wheelbase = carsJson[i]["轴距"];
        car.chassis.track = carsJson[i]["轮距"];
        car.chassis.min_ground_clearance = carsJson[i]["最小离地间隙"];
        car.chassis.min_turning_radius = carsJson[i]["最小转弯半径"];
        car.chassis.drive_type = carsJson[i]["驱动形式"];
        car.chassis.max_range = carsJson[i]["最大行程"];
        car.chassis.tire_model = carsJson[i]["轮胎型号"];
        car.chassis.tire_size = carsJson[i]["轮胎尺寸"];

        car.agx_kit.model = carsJson[i]["AGXKit型号"];
        car.agx_kit.ai = carsJson[i]["AI性能"];
        car.agx_kit.cuda_cores = carsJson[i]["CUDA核心数"];
        car.agx_kit.tensor_cores = carsJson[i]["Tensor核心数"];
        car.agx_kit.memory = carsJson[i]["内存大小"];
        car.agx_kit.storage = carsJson[i]["存储大小"];

        car.stereo_camera.model = carsJson[i]["立体相机型号"];
        car.stereo_camera.camera = carsJson[i]["相机型号"];
        car.stereo_camera.rgb_resolution = carsJson[i]["RGB分辨率"];
        car.stereo_camera.rgb_fps = carsJson[i]["RGB帧率"];
        car.stereo_camera.fov = carsJson[i]["视场角"];
        car.stereo_camera.depth_fps = carsJson[i]["深度帧率"];

        car.lidar.model = carsJson[i]["激光雷达型号"];
        car.lidar.channels = carsJson[i]["通道数"];
        car.lidar.range = carsJson[i]["测试范围"];
        car.lidar.power = carsJson[i]["功耗"];

        car.gyroscope.model = carsJson[i]["陀螺仪型号"];
        car.gyroscope.manufacturer = carsJson[i]["厂家"];
        
        car.lcd.size = carsJson[i]["液晶显示屏尺寸"];
        car.lcd.model = carsJson[i]["液晶显示屏型号"];

        car.battery.parameter = carsJson[i]["电池参数"];
        car.battery.output = carsJson[i]["对外供电"];

        cars.push_back(car);
    }
    ifs.close();
    return cars;
}





#endif // UTILS_H