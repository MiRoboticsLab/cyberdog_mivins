// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#pragma once

#include <stdio.h>
#include <string.h>
#include <iostream>

#define ANSI_COLOR_RED "\x1b[0;31m"
#define ANSI_COLOR_RED_BOLD "\x1b[1;31m"
#define ANSI_COLOR_RED_BG "\x1b[0;41m"

#define ANSI_COLOR_GREEN "\x1b[0;32m"
#define ANSI_COLOR_GREEN_BOLD "\x1b[1;32m"
#define ANSI_COLOR_GREEN_BG "\x1b[0;42m"

#define ANSI_COLOR_YELLOW "\x1b[0;33m"
#define ANSI_COLOR_YELLOW_BOLD "\x1b[1;33m"
#define ANSI_COLOR_YELLOW_BG "\x1b[0;43m"

#define ANSI_COLOR_BLUE "\x1b[0;34m"
#define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"
#define ANSI_COLOR_BLUE_BG "\x1b[0;44m"

#define ANSI_COLOR_MAGENTA "\x1b[0;35m"
#define ANSI_COLOR_MAGENTA_BOLD "\x1b[1;35m"
#define ANSI_COLOR_MAGENTA_BG "\x1b[0;45m"

#define ANSI_COLOR_CYAN "\x1b[0;36m"
#define ANSI_COLOR_CYAN_BOLD "\x1b[1;36m"
#define ANSI_COLOR_CYAN_BG "\x1b[0;46m"

#define ANSI_COLOR_WHITE "\x1b[0;37m"
#define ANSI_COLOR_WHITE_BOLD "\x1b[1;37m"
#define ANSI_COLOR_WHITE_BG "\x1b[0;47m"

#define ANSI_COLOR_BLACK "\x1b[0;30m"
#define ANSI_COLOR_BLACK_BOLD "\x1b[1;30m"
#define ANSI_COLOR_BLACK_BG "\x1b[0;40m"

#define ANSI_COLOR_RESET "\x1b[0m"

#define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
#define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
#define ANSI_SCREEN_FLUSH std::fflush(stdout);

#define SET_PRINT_COLOR( a ) cout << a ;

struct ScopeColor
{
  ScopeColor( const char * color )
  {
    std::cout << color;
  }

  ~ScopeColor()
  {
    std::cout << ANSI_COLOR_RESET;
  }
};

#define scope_color(color) ScopeColor scope(color);
