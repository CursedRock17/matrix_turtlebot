#ifndef TURTLEBOT4_CUSTOM__VISIBILITY_CONTROL_H_
#define TURTLEBOT4_CUSTOM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLEBOT4_CUSTOM_EXPORT __attribute__ ((dllexport))
    #define TURTLEBOT4_CUSTOM_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLEBOT4_CUSTOM_EXPORT __declspec(dllexport)
    #define TURTLEBOT4_CUSTOM_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLEBOT4_CUSTOM_BUILDING_LIBRARY
    #define TURTLEBOT4_CUSTOM_PUBLIC TURTLEBOT4_CUSTOM_EXPORT
  #else
    #define TURTLEBOT4_CUSTOM_PUBLIC TURTLEBOT4_CUSTOM_IMPORT
  #endif
  #define TURTLEBOT4_CUSTOM_PUBLIC_TYPE TURTLEBOT4_CUSTOM_PUBLIC
  #define TURTLEBOT4_CUSTOM_LOCAL
#else
  #define TURTLEBOT4_CUSTOM_EXPORT __attribute__ ((visibility("default")))
  #define TURTLEBOT4_CUSTOM_IMPORT
  #if __GNUC__ >= 4
    #define TURTLEBOT4_CUSTOM_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLEBOT4_CUSTOM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLEBOT4_CUSTOM_PUBLIC
    #define TURTLEBOT4_CUSTOM_LOCAL
  #endif
  #define TURTLEBOT4_CUSTOM_PUBLIC_TYPE
#endif

#endif  // TURTLEBOT4_CUSTOM__VISIBILITY_CONTROL_H_
