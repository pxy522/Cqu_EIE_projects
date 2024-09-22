#ifndef AGXKIT_H
#define AGXKIT_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;

class AGXKit
{
public:
    string model;
    string ai;
    int cuda_cores;
    int tensor_cores;
    int memory;
    int storage;

    void set()
    {
        cout << "请输入AGXKit型号: ";
        cin >> model;
        cout << "请输入AI性能: ";
        cin >> ai;
        cout << "请输入CUDA核心数: ";
        cin >> cuda_cores;
        cout << "请输入Tensor核心数: ";
        cin >> tensor_cores;
        cout << "请输入内存大小: ";
        cin >> memory;
        cout << "请输入存储大小: ";
        cin >> storage;
    }

    void print()
    {
        cout << setw(20) << "AGXKit型号: " << model << endl;
        cout << setw(20) << "AI性能: " << ai << endl;
        cout << setw(20) << "CUDA核心数: " << cuda_cores << endl;
        cout << setw(20) << "Tensor核心数: " << tensor_cores << endl;
        cout << setw(20) << "内存大小: " << memory << endl;
        cout << setw(20) << "存储大小: " << storage << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "AGX套件信息:" << endl;
        ofs << "        (a) 型号: " << model << endl;
        ofs << "        (b) AI: " << ai << endl;
        ofs << "        (c) CUDA核心: " << cuda_cores << endl;
        ofs << "        (d) Tensor CORE: " << tensor_cores << endl;
        ofs << "        (e) 显存: " << memory << endl;
        ofs << "        (f) 存储: " << storage << endl;
    }

};





#endif // AGXKIT_H