#pragma once
/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

// #define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

// IWYU pragma:  begin_keep
#include "api.h" 
#include "pros/apix.h" 
#include "liblvgl/lvgl.h"
// IWYU pragma:  end_keep

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#endif  // _PROS_MAIN_H_
