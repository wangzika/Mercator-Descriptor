// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
//这个文件定义了一个名为TicToc的类，用于计算程序运行时间。它包含了tic()和toc()两个函数，分别用于记录开始时间和结束时间，并计算时间差。
//其中，tic()函数记录当前时间作为开始时间，toc()函数记录当前时间作为结束时间，并计算时间差，最后输出时间差。这个类还包含了一个disp_变量，用于控制是否输出时间差。

#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    TicToc( bool _disp )
    {
        disp_ = _disp;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void toc( std::string _about_task )
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        if( disp_ )
        {
          std::cout.precision(3); // 10 for sec, 3 for ms 
          std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
        }
    }

private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool disp_ = false;
};
