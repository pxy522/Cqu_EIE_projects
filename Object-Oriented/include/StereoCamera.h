#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;


class StereoCamera
{
public:
    string model = "RealSense D435i";
    string camera = "D430";
    string rgb_resolution = "1920x1080";
    string rgb_fps = "30";
    string fov = "87x58";
    string depth_fps = "90";

    void set()
    {
        cout << "请输入立体相机型号: ";
        cin >> model;
        cout << "请输入相机型号: ";
        cin >> camera;
        cout << "请输入RGB分辨率: ";
        cin >> rgb_resolution;
        cout << "请输入RGB帧率: ";
        cin >> rgb_fps;
        cout << "请输入视场角: ";
        cin >> fov;
        cout << "请输入深度帧率: ";
        cin >> depth_fps;
    }

    void print()
    {
        cout << setw(20) << "立体相机型号: " << model << endl;
        cout << setw(20) << "相机型号: " << camera << endl;
        cout << setw(20) << "RGB分辨率: " << rgb_resolution << endl;
        cout << setw(20) << "RGB帧率: " << rgb_fps << endl;
        cout << setw(20) << "视场角: " << fov << endl;
        cout << setw(20) << "深度帧率: " << depth_fps << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "立体相机信息:" << endl;
        ofs << "        (a) 型号: " << model << endl;
        ofs << "        (b) 摄像头: " << camera << endl;
        ofs << "        (c) RGB帧分辨率: " << rgb_resolution << endl;
        ofs << "        (d) RGB帧率: " << rgb_fps << endl;
        ofs << "        (e) FOV: " << fov << endl;
        ofs << "        (f) 深度帧率: " << depth_fps << endl;
    }
};

#endif // STEREOCAMERA_H
