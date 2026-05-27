// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_INCLUDE_MJSAN_H_
#define MUJOCO_INCLUDE_MJSAN_H_

// Include asan interface header, or provide stubs for poison/unpoison macros when not using asan.
#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
#elif defined(_MSC_VER)
  #define ASAN_POISON_MEMORY_REGION(addr, size)
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size)
#else
  #define ASAN_POISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
#endif


#endif  // MUJOCO_INCLUDE_MJSAN_H_
