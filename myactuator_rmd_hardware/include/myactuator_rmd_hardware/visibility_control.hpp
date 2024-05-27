/**
 * \file visibility_control.hpp
 * \mainpage
 *    Auto-generated C++ header by "ros2 pkg create"
 *    This logic was borrowed (then namespaced) from the examples on the GCC wiki:
 *    https://gcc.gnu.org/wiki/Visibility
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD_HARDWARE__VISIBILITY_CONTROL
#define MYACTUATOR_RMD_HARDWARE__VISIBILITY_CONTROL
#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MYACTUATOR_RMD_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define MYACTUATOR_RMD_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define MYACTUATOR_RMD_HARDWARE_EXPORT __declspec(dllexport)
    #define MYACTUATOR_RMD_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MYACTUATOR_RMD_HARDWARE_BUILDING_LIBRARY
    #define MYACTUATOR_RMD_HARDWARE_PUBLIC MYACTUATOR_RMD_HARDWARE_EXPORT
  #else
    #define MYACTUATOR_RMD_HARDWARE_PUBLIC MYACTUATOR_RMD_HARDWARE_IMPORT
  #endif
  #define MYACTUATOR_RMD_HARDWARE_PUBLIC_TYPE MYACTUATOR_RMD_HARDWARE_PUBLIC
  #define MYACTUATOR_RMD_HARDWARE_LOCAL
#else
  #define MYACTUATOR_RMD_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define MYACTUATOR_RMD_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define MYACTUATOR_RMD_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define MYACTUATOR_RMD_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MYACTUATOR_RMD_HARDWARE_PUBLIC
    #define MYACTUATOR_RMD_HARDWARE_LOCAL
  #endif
  #define MYACTUATOR_RMD_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MYACTUATOR_RMD_HARDWARE__VISIBILITY_CONTROL
