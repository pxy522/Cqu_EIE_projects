/**
 * @file Subject.h
 * @author pxy522 (pxy174@gamil.com)
 * @brief 主题类
 * @version 0.1
 * @date 2024-09-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef SUBJECT_H
#define SUBJECT_H


#include <iostream>
#include <vector>
#include "Observer.h"

using namespace std;

class Subject
{
public:
    virtual ~Subject() {}

    void attach(Observer *observer)
    {
        m_observers.push_back(observer);
    }

    void detach(Observer *observer)
    {
        for (auto it = m_observers.begin(); it != m_observers.end(); it++)
        {
            if (*it == observer)
            {
                m_observers.erase(it);
                break;
            }
        }
    }

    void notify( const string& msg )
    {
        for (auto observer : m_observers)
        {
            observer->update( msg );
        }
    }

private:
    vector<Observer *> m_observers;
};

#endif // SUBJECT_H