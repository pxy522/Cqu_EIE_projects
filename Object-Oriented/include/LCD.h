#ifndef LCD_H
#define LCD_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

using namespace std;


class LCD
{
public:
    string size = "11.6";
    string model = "super";

    void set()
    {
        cout << "请输入液晶显示屏尺寸: ";
        cin >> size;
        cout << "请输入液晶显示屏型号: ";
        cin >> model;
    }

    void print()
    {
        cout << setw(20) << "液晶显示屏尺寸: " << size << endl;
        cout << setw(20) << "液晶显示屏型号: " << model << endl;
    }

    void save( ofstream& ofs )
    {
        ofs << "液晶显示屏信息:" << endl;
        ofs << "        (a) 尺寸: " << size << endl;
        ofs << "        (b) 型号: " << model << endl;
    }
};


#endif // LCD_H