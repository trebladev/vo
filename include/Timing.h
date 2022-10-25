//
// Created by xuan on 10/24/22.
//

#ifndef VO_INCLUDE_TIMING_H_
#define VO_INCLUDE_TIMING_H_

#include <chrono>
#include <iostream>

#define TIMER_START(NAME) \
std::chrono::steady_clock::time_point __TIMER##NAME##_START = std::chrono::steady_clock::now();


#define TIMER_END(NAME) \
std::chrono::steady_clock::time_point __TIMER##NAME##_END = std::chrono::steady_clock::now(); \
{ \
auto d = std::chrono::duration_cast<std::chrono::duration<float>>( __TIMER##NAME##_END - __TIMER##NAME##_START ).count(); \
printf("Timer " #NAME ": %f s\n",d);             \
}

#endif //VO_INCLUDE_TIMING_H_
