//container: macro, header file

//为了避免同一个头文件被包含（include）多次，C/C++中有两种宏实现方式：一种是#ifndef方式；另一种是#pragma once方式。
#pragma once
/*
1.pragma once 一般由编译器提供保证：同一个文件不会被包含多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。
2.你无法对一个头文件中的一段代码作pragma once声明，而只能针对文件。
3.不必担心宏名冲突了，也就会出现宏名冲突引发的问题。大型项目的编译速度也因此提高了一些。
4.缺点就是如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。当然，相比宏名冲突引发的“找不到声明”的问题，这种重复包含很容易被发现并修正。
5.这种方式不支持跨平台,不受一些较老版本的编译器支持，
*/
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
/*
1.ifndef的方式受C/C++语言标准支持。它不仅可以保证同一个文件不会被包含多次，也能保证内容完全相同的两个文件（或者代码片段）不会被不小心同时包含。
2.当然，缺点就是如果不同头文件中的宏名不小心“撞车”，可能就会导致你看到头文件明明存在，但编译器却硬说找不到声明的状况——这种情况有时非常让人郁闷。
3.由于编译器每次都需要打开头文件才能判定是否有重复定义，因此在编译大型项目时，ifndef会使得编译时间相对较长，因此一些编译器逐渐开始支持#pragma once的方式。
4.受C/C++语言标准的支持，不受编译器的任何限制；
*/
#define _UTILITY_LIDAR_ODOMETRY_H_ // define macro behind ifndef

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

/*
macro:
  FLT_MAX: max positive float
*/


#endif
