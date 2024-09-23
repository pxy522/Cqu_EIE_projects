/**
 * @file Observer.h
 * @author pxy522 (pxy174@gamil.com)
 * @brief 抽象观察者类
 * @version 0.1
 * @date 2024-09-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef OBSERVER_H
#define OBSERVER_H

using namespace std;

class Observer
{
public:
    virtual ~Observer() {}

    virtual void update( const string& msg ) = 0;
};

#endif // OBSERVER_H