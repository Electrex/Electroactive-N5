/*
 * Copyright 2015 Park Ju Hyung <qkrwngud825@gmail.com>
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

/*
 * Adreno idler - Idling algorithm,
 * an efficient workaround for msm-adreno-tz's overheads. (Modified framework to work with kgsl)
 *
 * Main goal is to lower the power consumptions while maintaining high-performance.
 *
 * Since msm-adreno-tz tends to *not* use the lowest frequency even on idle,
 * Adreno idler replaces msm-adreno-tz's algorithm when it comes to
 * calculating idle frequency(mostly by ondemand's method).
 * The higher frequencies are not touched with this algorithm, so high-demanding
 * games will (most likely) not suffer from worsened performance.
 */

#include <linux/module.h>

#include "adreno_idler.h"
#include "kgsl_device.h"
#include "kgsl_pwrscale.h"

/*
 * stats.busy_time threshold for determining if the given workload is idle.
 * Any workload higher than this will be treated as a non-idle workload.
 * Adreno idler will more actively try to ramp down the frequency
 * if this is set to a higher value.
 */
static unsigned long idleworkload = 10000;
module_param_named(adreno_idler_idleworkload, idleworkload, ulong, 0664);

/*
 * Number of events to wait before ramping down the frequency.
 * The idlewait'th events before current one must be all idle before
 * Adreno idler ramps down the frequency.
 * This implementation is to prevent micro-lags on scrolling or playing games.
 * Adreno idler will more actively try to ramp down the frequency
 * if this is set to a lower value.
 */
static unsigned int idlewait = 15;
module_param_named(adreno_idler_idlewait, idlewait, uint, 0664);

static bool adreno_idler_active = true;
module_param_named(adreno_idler_active, adreno_idler_active, bool, 0664);

int adreno_idler(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_power_stats stats;
	unsigned int idlecount = 0;

	if (!adreno_idler_active)
		return 0;

	device->ftbl->power_stats(device, &stats);

	if (stats.busy_time < idleworkload) {
		/*
		 * busy_time >= idleworkload should be considered as a non-idle
		 * workload.
		 */
		idlecount++;
		if (pwr->active_pwrlevel == 2)
			return 1;

		if (idlecount >= idlewait) {
			/*
			 * We are idle for (idlewait + 1)'th time!
			 * Ramp down the frequency now. 
			 */
			kgsl_pwrctrl_pwrlevel_change(device, 2);
			idlecount--;
			return 1;
		}
	} else {
		/* Reset idlecount */
		if (idlecount > 0)
			idlecount = 0;
	}

	return 0;
}

static int __init adreno_idler_init(void)
{
	pr_info("adreno_idler: initialized!\n");

	return 0;
}
subsys_initcall(adreno_idler_init);

static void __exit adreno_idler_exit(void)
{
	return;
}
module_exit(adreno_idler_exit);

MODULE_AUTHOR("Park Ju Hyung <qkrwngud825@gmail.com>");
MODULE_DESCRIPTION("'adreno_idler - a powersaver for the tz algorithm");
MODULE_LICENSE("GPL");
