#ifndef TIME_H
#define TIME_H

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
//#include <ros/ros.h>
//#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace seok {

    class TimeChecker
    {
    public:
        TimeChecker(){ t1 = std::chrono::system_clock::now(); }

        void interval(std::string str)
        {
            t2 = std::chrono::system_clock::now();
            std::chrono::duration<double> dur = t2 - t1;

            std::cout << "Time taken for " << str  << " : " << dur.count() << " s" << std::endl;
            //ROS_INFO("Time taken for %s : %lfs", str.c_str() , dur.count());
        }

        std::string returnInterval(std::string str){
            t2 = std::chrono::system_clock::now();
            std::chrono::duration<double> dur = t2 - t1;

            std::cout << "Time taken for " << str  << " : " << dur.count() << " s" << std::endl;
            
            return "Time taken for " + str  + " : " + std::to_string(dur.count()) + " s\n";  
        }



    private:
        std::chrono::time_point<std::chrono::system_clock> t1;
        std::chrono::time_point<std::chrono::system_clock> t2;
    };
}
#endif 
