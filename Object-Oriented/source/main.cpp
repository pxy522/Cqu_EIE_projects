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

#include <iostream>

#include "Cars.h"
#include "utils.h"

using namespace std;


int main()
{
    cout << "********************************" << endl;
    cout << "       >> 1.观察者模式<<" << endl;
    cout << "      >> 2.小车信息管理<<" << endl;
    cout << "********************************" << endl;
    cout << endl;

    cout << "请选择操作: ";
    int choice;
    cin >> choice;
    cout << endl;

    switch (choice){
        case 1:{
            string response[] {"前方", "右前方", "左前方"};
            /* 观察者模式 */
            Chassis chassis;       // 观察者
            Lidar lidar;           // 主题
            lidar.attach(&chassis);
            cout << "测试激光雷达和地盘通信" << endl;
            cout << "--------------------------------" << endl;
            cout << endl;
            while ( true )
            {
                cout << "请设置障碍物状态 '1.前方'、'2.右前方'或'3.左前方': ";
                int msg;
                cin >> msg;
                if (msg == 0){
                    cout << "!!THANKS!!" << endl;
                    break;
                }
                cout << endl;
                cout << "<<<<<<<<<<<<<<<<<" << endl;
                lidar.notify(response[msg-1]);
                cout << ">>>>>>>>>>>>>>>>>" << endl;
                cout << endl;
                cout << "输入 '0' 退出 " << endl;
                cout << endl;

            }
            break;
        }
        case 2:
        {
        vector<Car> cars;
        /* 循环 */
        while ( true )
        {
            cout << "--------------------------------" << endl;
            cout << "1.录入车辆信息" << endl;
            cout << "2.输出车辆信息" << endl;
            cout << "3.修改车辆信息" << endl;
            cout << "4.保存车辆信息" << endl;
            cout << "5.从文件中读取车辆信息" << endl;
            cout << "6.退出" << endl;
            cout << "--------------------------------" << endl;
            cout << "请选择操作: ";
            int choice;
            cin >> choice;
            cout << endl;
            switch (choice)
            {
            case 1:
                {
                    if (cars.size() >= 10)
                    {
                        cout << "*****************"<< endl;
                        cout << ">>车辆信息已满!<<" << endl;
                        cout << "*****************"<< endl;
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
                {
                int idx = 0;
                char c, choice;
                while ( true )
                {
                    cars[idx].print();
                    cout << "查看详细信息? (y/n): ";

                    cin >> choice;
                    if (choice == 'y')
                    {
                        cars[idx].chassis.print();
                        cars[idx].agx_kit.print();
                        cars[idx].stereo_camera.print();
                        cars[idx].lidar.print();
                        cars[idx].gyroscope.print();
                        cars[idx].lcd.print();
                        cars[idx].battery.print();
                    }

                    cout << "输入 'n' 查看下一辆车, 'p' 查看上一辆车, 'q' 退出: ";
                    cin >> c;
                    cout << endl;
                    if (c == 'n')
                        idx = (idx + 1) % cars.size();
                    else if (c == 'p')
                        idx = (idx - 1 + cars.size()) % cars.size();
                    else if (c == 'q')
                        break;
                    }
                    cout << endl;   
                    break;
                }
            case 3:
                modifyCarInfo(cars);
                break;
            case 4:
                {
                    string path;
                    cout << "请输入保存路径: ";
                    cin >> path;
                    ofstream ofs(path);
                    
                    json carsJson;
                    for (int i = 0; i < cars.size(); i++)
                    {   
                        string id = "车辆" + to_string(i + 1);
                        carsJson[id] = cars[i].save();
                    }
                    ofs << carsJson.dump(4);
                    break;
                }
            case 5:
                {
                    string path;
                    cout << "请输入文件路径: ";
                    cin >> path;
                    cout << endl;
                    cars = readJsonInfo(path);
                    break;
                }
            case 6:
                return 0;
            default:
                break;
            }
        }
    }
    }
    return 0;
}
