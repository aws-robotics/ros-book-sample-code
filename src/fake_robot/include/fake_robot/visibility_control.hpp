// Copyright (c) 2020 Amazon.com, Inc. or its affiliates
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef FAKE_ROBOT__VISIBILITY_CONTROL_HPP_
#define FAKE_ROBOT__VISIBILITY_CONTROL_HPP_

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
#endif  // FAKE_ROBOT__VISIBILITY_CONTROL_HPP_
