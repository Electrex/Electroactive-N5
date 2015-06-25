/*
 * Copyright 2015 Tom G. <roboter972@gmail.com>
 * Copyright 2015 Yusuf Mostafa <ymostafa30@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ADRENO_IDLER_H
#define __ADRENO_IDLER_H

#include "kgsl_device.h"
#include "kgsl_pwrscale.h"

int adreno_idler(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale);

#endif /* __ADRENO_IDLER_H */
