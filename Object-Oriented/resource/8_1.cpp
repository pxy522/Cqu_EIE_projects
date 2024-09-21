#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>
#include <sstream>

using namespace std;

struct Chassis {
    string id;
    string model = "SCOUT MINI";
    int wheelbase = 451;
    int track = 490;
    int min_ground_clearance = 115;
    int min_turning_radius = 0;
    string drive_type = "四轮四驱";
    int max_range = 10;
    string tire_model = "公路轮、麦克纳姆轮";
    int tire_size = 175;
};

struct AGXKit {
    string model = "AGX Xavier";
    string ai = "32 TOPS";
    int cuda_cores = 512;
    int tensor_cores = 64;
    int memory = 32;
    int storage = 32;
};

struct StereoCamera {
    string model = "RealSense D435i";
    string camera = "D430";
    string rgb_resolution = "1920x1080";
    int rgb_fps = 30;
    string fov = "87x58";
    int depth_fps = 90;
};

struct Lidar {
    string model = "RS-Helios-16p";
    int channels = 16;
    int range = 100;
    int power = 8;
};

struct Gyroscope {
    string model = "CH110";
    string manufacturer = "NXP";
};

struct LCD {
    float size = 11.6f;
    string model = "super";
};

struct Battery {
    string parameter = "24V/15Ah";
    string output = "24V";
    int charge_time = 2;
};

struct Car {
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
};

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

void inputCarInfo(Car& car)
{
    cout << "请输入学号: ";
    cin >> car.student_id;
    cout << "请输入姓名: ";
    cin >> car.student_name;
    cout << "生成车辆ID..." << endl;
    car.id = "cqusn" + getCurrentTime() + car.student_id;
    car.chassis.id = "dp" + car.student_id;
}

void save2file(const vector<Car>& cars, const string& filename)
{
    ofstream ifs(filename);
    for (const auto& car : cars)
    {
        ifs << "--------" << car.id << "--------(" << car.student_id << ")" << endl;
        ifs << "车辆信息:" << endl;
        ifs << "        (a) 底盘编号: " << car.chassis.id << endl;
        ifs << "        (b) 底盘型号: " << car.chassis.model << endl;
        ifs << "        (c) 轴距: " << car.chassis.wheelbase << endl;
        ifs << "        (d) 轮距: " << car.chassis.track << endl;
        ifs << "        (e) 最小离地间隙: " << car.chassis.min_ground_clearance << endl;
        ifs << "        (f) 最小转弯半径: " << car.chassis.min_turning_radius << endl;
        ifs << "        (g) 驱动形式: " << car.chassis.drive_type << endl;
        ifs << "        (h) 最大行程: " << car.chassis.max_range << endl;
        ifs << "        (i) 轮胎型号: " << car.chassis.tire_model << endl;
        ifs << "        (j) 轮胎尺寸: " << car.chassis.tire_size << endl;
        ifs << "AGX套件信息:" << endl;
        ifs << "        (a) 型号: " << car.agx_kit.model << endl;
        ifs << "        (b) AI: " << car.agx_kit.ai << endl;
        ifs << "        (c) CUDA核心: " << car.agx_kit.cuda_cores << endl;
        ifs << "        (d) Tensor CORE: " << car.agx_kit.tensor_cores << endl;
        ifs << "        (e) 显存: " << car.agx_kit.memory << endl;
        ifs << "        (f) 存储: " << car.agx_kit.storage << endl;
        ifs << "双目摄像头信息:" << endl;
        ifs << "        (a) 型号: " << car.stereo_camera.model << endl;
        ifs << "        (b) 摄像头: " << car.stereo_camera.camera << endl;
        ifs << "        (c) RGB帧分辨率: " << car.stereo_camera.rgb_resolution << endl;
        ifs << "        (d) RGB帧率: " << car.stereo_camera.rgb_fps << endl;
        ifs << "        (e) FOV: " << car.stereo_camera.fov << endl;
        ifs << "        (f) 深度帧率: " << car.stereo_camera.depth_fps << endl;
        ifs << "多线激光雷达信息:" << endl;
        ifs << "        (a) 型号: " << car.lidar.model << endl;
        ifs << "        (b) 通道数: " << car.lidar.channels << endl;
        ifs << "        (c) 测试范围: " << car.lidar.range << endl;
        ifs << "        (d) 功耗: " << car.lidar.power << endl;
        ifs << "9轴陀螺仪信息:" << endl;
        ifs << "        (a) 型号: " << car.gyroscope.model << endl;
        ifs << "        (b) 厂家: " << car.gyroscope.manufacturer << endl;
        ifs << "液晶显示屏信息:" << endl;
        ifs << "        (a) 尺寸: " << car.lcd.size << endl;
        ifs << "        (b) 型号: " << car.lcd.model << endl;
        ifs << "电池模块信息:" << endl;
        ifs << "        (a) 参数: " << car.battery.parameter << endl;
        ifs << "        (b) 对外供电: " << car.battery.output << endl;
        ifs << "        (c) 充电时长: " << car.battery.charge_time << endl;
		ifs << "学生信息:" << endl;
		ifs << "        (a) 学号: " << car.student_id << endl;
		ifs << "        (b) 学生姓名: " << car.student_name << endl;
    }
}


void displayInfo(const vector<Car>& cars, int idx)
{
    cout << "--------------------------------" << endl;
    cout << "车辆ID: " << cars[idx].id << endl;
    cout << "底盘编号: " << cars[idx].chassis.id << endl;
    cout << "学号: " << cars[idx].student_id << " 姓名: " << cars[idx].student_name << endl;
    cout << "--------------------------------" << endl;
}

int main()
{
    vector<Car> cars(10);

    for (auto& car : cars)
    {
        inputCarInfo(car);
    }

    string filename = "car.txt";
    save2file(cars, filename);

    char c;
    int idx = 0;

    while (true)
    {
        displayInfo(cars, idx);
        cout << "输入 'n' 查看下一辆车, 'p' 查看上一辆车, 'q' 退出: ";
        cout << endl;

        cin >> c;
        if (c == 'n')
            idx = (idx + 1) % cars.size();
        else if (c == 'p')
            idx = (idx - 1 + cars.size()) % cars.size();
        else if (c == 'q')
            break;
    }

    return 0;
}