#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>
#include <sstream>

using namespace std;

class Chassis {
public:
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

    void set(){
        
    }
};

class AGXKit {
public:
    string model = "AGX Xavier";
    string ai = "32 TOPS";
    int cuda_cores = 512;
    int tensor_cores = 64;
    int memory = 32;
    int storage = 32;
};

class StereoCamera {
public:
    string model = "RealSense D435i";
    string camera = "D430";
    string rgb_resolution = "1920x1080";
    int rgb_fps = 30;
    string fov = "87x58";
    int depth_fps = 90;
};

class Lidar {
public:
    string model = "RS-Helios-16p";
    int channels = 16;
    int range = 100;
    int power = 8;
};

class Gyroscope {
public:
    string model = "CH110";
    string manufacturer = "NXP";
};

class LCD {
public:
    float size = 11.6f;
    string model = "super";
};

class Battery {
public:
    string parameter = "24V/15Ah";
    string output = "24V";
    int charge_time = 2;
};

class Car {
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

    void inputCarInfo() {
        cout << "请输入学号: ";
        cin >> student_id;
        cout << "请输入姓名: ";
        cin >> student_name;
        cout << "生成车辆ID..." << endl;
        id = "cqusn" + getCurrentTime() + student_id;
        chassis.id = "dp" + student_id;
    }

    void displayInfo() {
        cout << "--------------------------------" << endl;
        cout << "车辆ID: " << id << endl;
        cout << "底盘编号: " << chassis.id << endl;
        cout << "学号: " << student_id << " 姓名: " << student_name << endl;
        cout << "--------------------------------" << endl;
    }

    string getCurrentTime() {
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
};

void save2file(const vector<Car>& cars, const string& filename) {
    // Same as before
}

int main() {
    vector<Car> cars(10);

    for (auto& car : cars) {
        car.inputCarInfo();
    }

    string filename = "car.txt";
    save2file(cars, filename);

    char c;
    int idx = 0;

    while (true) {
        cars[idx].displayInfo();
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