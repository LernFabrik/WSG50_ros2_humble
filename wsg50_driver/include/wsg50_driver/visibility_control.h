// Copyright 2026 WSG50 ROS maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WSG50_DRIVER__VISIBILITY_CONTROL_H_
#define WSG50_DRIVER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WSG50_DRIVER_EXPORT __attribute__((dllexport))
    #define WSG50_DRIVER_IMPORT __attribute__((dllimport))
  #else
    #define WSG50_DRIVER_EXPORT __declspec(dllexport)
    #define WSG50_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef WSG50_DRIVER_BUILDING_DLL
    #define WSG50_DRIVER_PUBLIC WSG50_DRIVER_EXPORT
  #else
    #define WSG50_DRIVER_PUBLIC WSG50_DRIVER_IMPORT
  #endif
#else
  #define WSG50_DRIVER_EXPORT __attribute__((visibility("default")))
  #define WSG50_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define WSG50_DRIVER_PUBLIC __attribute__((visibility("default")))
  #else
    #define WSG50_DRIVER_PUBLIC
  #endif
#endif

#endif  // WSG50_DRIVER__VISIBILITY_CONTROL_H_
