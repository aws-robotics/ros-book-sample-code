// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#ifndef FAKE_ROBOT__VISIBILITY_CONTROL_H_
#define FAKE_ROBOT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FAKE_ROBOT_EXPORT __attribute__ ((dllexport))
    #define FAKE_ROBOT_IMPORT __attribute__ ((dllimport))
  #else
    #define FAKE_ROBOT_EXPORT __declspec(dllexport)
    #define FAKE_ROBOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef FAKE_ROBOT_BUILDING_DLL
    #define FAKE_ROBOT_PUBLIC FAKE_ROBOT_EXPORT
  #else
    #define FAKE_ROBOT_PUBLIC FAKE_ROBOT_IMPORT
  #endif
  #define FAKE_ROBOT_PUBLIC_TYPE FAKE_ROBOT_PUBLIC
  #define FAKE_ROBOT_LOCAL
#else
  #define FAKE_ROBOT_EXPORT __attribute__ ((visibility("default")))
  #define FAKE_ROBOT_IMPORT
  #if __GNUC__ >= 4
    #define FAKE_ROBOT_PUBLIC __attribute__ ((visibility("default")))
    #define FAKE_ROBOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FAKE_ROBOT_PUBLIC
    #define FAKE_ROBOT_LOCAL
  #endif
  #define FAKE_ROBOT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // FAKE_ROBOT__VISIBILITY_CONTROL_H_
