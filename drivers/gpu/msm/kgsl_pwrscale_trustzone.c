/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/socinfo.h>
#include <mach/scm.h>

#include "adreno_idler.h"
#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"

#ifdef CONFIG_MSM_KGSL_MSM_ADRENO_TZ
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/ftrace.h>
#include <linux/msm_adreno_devfreq.h>
#include "governor.h"

extern bool mdss_screen_on;

#endif

#define TZ_GOVERNOR_PERFORMANCE 0
#define TZ_GOVERNOR_ONDEMAND    1
#ifdef CONFIG_MSM_KGSL_MSM_ADRENO_TZ
#define TZ_GOVERNOR_MSM_ADRENO_TZ 2
#endif

struct tz_priv {
	int governor;
	struct kgsl_power_stats bin;
	unsigned int idle_dcvs;
};
spinlock_t tz_lock;

/* FLOOR is 5msec to capture up to 3 re-draws
 * per frame for 60fps content.
 */
#define FLOOR			5000
/* CEILING is 50msec, larger than any standard
 * frame length, but less than the idle timer.
 */
#define CEILING			50000
#define TZ_RESET_ID		0x3
#define TZ_UPDATE_ID		0x4
#define TZ_INIT_ID		0x6

#define TAG "msm_adreno_tz: "

/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_entry2(u32 cmd, u32 val1, u32 val2)
{
	int ret;
	spin_lock(&tz_lock);
	/* sync memory before sending the commands to tz*/
	__iowmb();
	ret = scm_call_atomic2(SCM_SVC_IO, cmd, val1, val2);
	spin_unlock(&tz_lock);
	return ret;
}

static int __secure_tz_entry3(u32 cmd, u32 val1, u32 val2,
				u32 val3)
{
	int ret;
	spin_lock(&tz_lock);
	/* sync memory before sending the commands to tz*/
	__iowmb();
	ret = scm_call_atomic3(SCM_SVC_IO, cmd, val1, val2,
				val3);
	spin_unlock(&tz_lock);
	return ret;
}

static ssize_t tz_governor_show(struct kgsl_device *device,
				struct kgsl_pwrscale *pwrscale,
				char *buf)
{
	struct tz_priv *priv = pwrscale->priv;
	int ret;

	if (priv->governor == TZ_GOVERNOR_ONDEMAND)
		ret = snprintf(buf, 10, "ondemand\n");
#ifdef CONFIG_MSM_KGSL_MSM_ADRENO_TZ
	else if (priv->governor == TZ_GOVERNOR_MSM_ADRENO_TZ)
		ret = snprintf(buf, 8, "msm-adreno-tz\n");
#endif
	else
		ret = snprintf(buf, 13, "performance\n");

	return ret;
}

static ssize_t tz_governor_store(struct kgsl_device *device,
				struct kgsl_pwrscale *pwrscale,
				 const char *buf, size_t count)
{
	char str[20];
	struct tz_priv *priv = pwrscale->priv;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int ret;

	ret = sscanf(buf, "%20s", str);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&device->mutex);

	if (!strncmp(str, "ondemand", 8))
		priv->governor = TZ_GOVERNOR_ONDEMAND;
#ifdef CONFIG_MSM_KGSL_MSM_ADRENO_TZ
	else if (!strncmp(str, "msm-adreno-tz", 6))
		priv->governor = TZ_GOVERNOR_MSM_ADRENO_TZ;
#endif
	else if (!strncmp(str, "performance", 11))
		priv->governor = TZ_GOVERNOR_PERFORMANCE;

	if (priv->governor == TZ_GOVERNOR_PERFORMANCE) {
		kgsl_pwrctrl_pwrlevel_change(device, pwr->max_pwrlevel);
		pwr->default_pwrlevel = pwr->max_pwrlevel;
	} else {
		pwr->default_pwrlevel = pwr->init_pwrlevel;
	}

	mutex_unlock(&device->mutex);
	return count;
}

PWRSCALE_POLICY_ATTR(governor, 0644, tz_governor_show, tz_governor_store);

static struct attribute *tz_attrs[] = {
	&policy_attr_governor.attr,
	NULL
};

static struct attribute_group tz_attr_group = {
	.attrs = tz_attrs,
};

static void tz_wake(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	return;
}

#ifdef CONFIG_MSM_KGSL_MSM_ADRENO_TZ
/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef CONFIG_ADRENO_IDLER
extern int adreno_idler(struct devfreq_dev_status stats, struct devfreq *devfreq,
		 unsigned long *freq);
#endif
static int tz_get_target_freq(struct devfreq *devfreq, unsigned long *freq,
				u32 *flag)
{
	int result = 0;
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	struct devfreq_dev_status stats;
	int val, level = 0;

	result = devfreq->profile->get_dev_status(devfreq->dev.parent, &stats);
	if (result) {
		pr_err(TAG "get_status failed %d\n", result);
		return result;
	}

        /* Prevent overflow */
	if (stats.busy_time >= (1 << 24) || stats.total_time >= (1 << 24)) {
		stats.busy_time >>= 7;
		stats.total_time >>= 7;
	}

	*freq = stats.current_frequency;

        /*
	 * Force to use & record as min freq when system has
	 * entered pm-suspend or screen-off state.
	 */
	if (!mdss_screen_on) {
		*freq = devfreq->profile->freq_table[devfreq->profile->max_state - 1];
		return 0;
	}
        
	priv->bin.total_time += stats.total_time;
	priv->bin.busy_time += stats.busy_time;
	/*
	 * Do not waste CPU cycles running this algorithm if
	 * the GPU just started, or if less than FLOOR time
	 * has passed since the last run.
	 */
	if ((stats.total_time == 0) ||
		(priv->bin.total_time < FLOOR)) {
		return 0;
	}

	level = devfreq_get_freq_level(devfreq, stats.current_frequency);
	if (level < 0) {
		pr_err(TAG "bad freq %ld\n", stats.current_frequency);
		return level;
	}

	/*
	 * If there is an extended block of busy processing,
	 * increase frequency.  Otherwise run the normal algorithm.
	 */
	if (priv->bin.busy_time > CEILING) {
		val = -1;
	} else {
		val = __secure_tz_entry3(TZ_UPDATE_ID,
				level,
				priv->bin.total_time,
				priv->bin.busy_time);
	}
	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;

	/*
	 * If the decision is to move to a lower level, make sure the GPU
	 * frequency drops.
	 */
	level += val;
	level = max(level, 0);
	level = min_t(int, level, devfreq->profile->max_state);
	*freq = devfreq->profile->freq_table[level];

	/*
	 * By setting freq as UINT_MAX we notify the kgsl target function
	 * to go up one power level without considering the freq value
	 */
	if (val < 0)
		*freq = UINT_MAX;

	return 0;
}

static int tz_notify(struct notifier_block *nb, unsigned long type, void *devp)
{
	int result = 0;
	struct devfreq *devfreq = devp;

	switch (type) {
	case ADRENO_DEVFREQ_NOTIFY_IDLE:
	case ADRENO_DEVFREQ_NOTIFY_RETIRE:
		mutex_lock(&devfreq->lock);
		result = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		break;
	/* ignored by this governor */
	case ADRENO_DEVFREQ_NOTIFY_SUBMIT:
	default:
		break;
	}
	return notifier_from_errno(result);
}

static int tz_start(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv;
	unsigned int tz_pwrlevels[MSM_ADRENO_MAX_PWRLEVELS + 1];
	int i, out, ret;

	if (devfreq->data == NULL) {
		pr_err(TAG "data is required for this governor\n");
		return -EINVAL;
	}

	priv = devfreq->data;
	priv->nb.notifier_call = tz_notify;

	out = 1;
	if (devfreq->profile->max_state < MSM_ADRENO_MAX_PWRLEVELS) {
		for (i = 0; i < devfreq->profile->max_state; i++)
			tz_pwrlevels[out++] = devfreq->profile->freq_table[i];
		tz_pwrlevels[0] = i;
	} else {
		pr_err(TAG "tz_pwrlevels[] is too short\n");
		return -EINVAL;
	}

	ret = scm_call(SCM_SVC_DCVS, TZ_INIT_ID, tz_pwrlevels,
			sizeof(tz_pwrlevels), NULL, 0);

	if (ret != 0)
		pr_err(TAG "tz_init failed\n");

	return kgsl_devfreq_add_notifier(devfreq->dev.parent, &priv->nb);
}

static int tz_stop(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;

	kgsl_devfreq_del_notifier(devfreq->dev.parent, &priv->nb);
	return 0;
}


static int tz_resume(struct devfreq *devfreq)
{
	struct devfreq_dev_profile *profile = devfreq->profile;
	unsigned long freq;

	freq = profile->initial_freq;

	return profile->target(devfreq->dev.parent, &freq,
				DEVFREQ_FLAG_LEAST_UPPER_BOUND);
}

static int tz_suspend(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;

	__secure_tz_entry2(TZ_RESET_ID, 0, 0);

	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;
	return 0;
}

static int tz_handler(struct devfreq *devfreq, unsigned int event, void *data)
{
	int result;
	BUG_ON(devfreq == NULL);

	switch (event) {
	case DEVFREQ_GOV_START:
		result = tz_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		result = tz_stop(devfreq);
		break;

	case DEVFREQ_GOV_SUSPEND:
		result = tz_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		result = tz_resume(devfreq);
		break;

	case DEVFREQ_GOV_INTERVAL:
		/* ignored, this governor doesn't use polling */
	default:
		result = 0;
		break;
	}

	return result;
}

static struct devfreq_governor msm_adreno_tz = {
	.name = "msm-adreno-tz",
	.get_target_freq = tz_get_target_freq,
	.event_handler = tz_handler,
};

static int __init msm_adreno_tz_init(void)
{
	return devfreq_add_governor(&msm_adreno_tz);
}
subsys_initcall(msm_adreno_tz_init);

static void __exit msm_adreno_tz_exit(void)
{
	int ret;
	ret = devfreq_remove_governor(&msm_adreno_tz);
	if (ret)
		pr_err(TAG "failed to remove governor %d\n", ret);

	return;
}

module_exit(msm_adreno_tz_exit);

MODULE_LICENSE("GPLv2");
#endif

static void tz_idle(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct tz_priv *priv = pwrscale->priv;
	struct kgsl_power_stats stats;
	int val, idle;

	/* In "performance" mode the clock speed always stays
	   the same */
	if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
		return;

	device->ftbl->power_stats(device, &stats);
	priv->bin.total_time += stats.total_time;
	priv->bin.busy_time += stats.busy_time;
	/* Do not waste CPU cycles running this algorithm if
	 * the GPU just started, or if less than FLOOR time
	 * has passed since the last run.
	 */
	if ((stats.total_time == 0) || (priv->bin.total_time < FLOOR))
		return;

	/* Adreno_idler has asked to bail out now. */
	if (adreno_idler(device, pwrscale) && priv->bin.busy_time <= CEILING)
		return;

	/* If there is an extended block of busy processing, set
	 * frequency to turbo.  Otherwise run the normal algorithm.
	 */
	if (priv->bin.busy_time > CEILING) {
		val = 0;
		kgsl_pwrctrl_pwrlevel_change(device,
				KGSL_PWRLEVEL_TURBO);
	} else if (priv->idle_dcvs) {
		idle = priv->bin.total_time - priv->bin.busy_time;
		idle = (idle > 0) ? idle : 0;
		val = __secure_tz_entry2(TZ_UPDATE_ID, idle, device->id);
	} else {
		if (pwr->step_mul > 1)
			val = __secure_tz_entry3(TZ_UPDATE_ID,
				(pwr->active_pwrlevel + 1)/2,
				priv->bin.total_time, priv->bin.busy_time);
		else
			val = __secure_tz_entry3(TZ_UPDATE_ID,
				pwr->active_pwrlevel,
				priv->bin.total_time, priv->bin.busy_time);
	}

	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;

	/* If the decision is to move to a lower level, make sure the GPU
	 * frequency drops.
	 */
	if (val > 0)
		val *= pwr->step_mul;

	if ((pwr->constraint.type == KGSL_CONSTRAINT_NONE) ||
			(time_after(jiffies, pwr->constraint.expires))) {

		kgsl_pwrctrl_pwrlevel_change(device,
					     pwr->active_pwrlevel + val);
		if (pwr->constraint.type != KGSL_CONSTRAINT_NONE) {
			/* Trace the constraint being un-set by the driver */
			trace_kgsl_constraint(device,
				pwr->constraint.type,
				pwr->active_pwrlevel, 0);
			/*Invalidate the constraint set */
			pwr->constraint.type = KGSL_CONSTRAINT_NONE;
		}
	}
}

static void tz_busy(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	device->on_time = ktime_to_us(ktime_get());
}

static void tz_sleep(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	struct tz_priv *priv = pwrscale->priv;

	kgsl_pwrctrl_pwrlevel_change(device, 2);
	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;
}

#ifdef CONFIG_MSM_SCM
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	int i = 0, j = 1, ret = 0;
	struct tz_priv *priv;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	unsigned int tz_pwrlevels[KGSL_MAX_PWRLEVELS + 1];

	priv = pwrscale->priv = kzalloc(sizeof(struct tz_priv), GFP_KERNEL);
	if (pwrscale->priv == NULL)
		return -ENOMEM;
	priv->idle_dcvs = 0;
	priv->governor = TZ_GOVERNOR_ONDEMAND;
	spin_lock_init(&tz_lock);
	kgsl_pwrscale_policy_add_files(device, pwrscale, &tz_attr_group);
	for (i = 0; i < pwr->num_pwrlevels - 1; i++) {
		if (i == 0)
			tz_pwrlevels[j] = pwr->pwrlevels[i].gpu_freq;
		else if (pwr->pwrlevels[i].gpu_freq !=
				pwr->pwrlevels[i - 1].gpu_freq) {
			j++;
			tz_pwrlevels[j] = pwr->pwrlevels[i].gpu_freq;
		}
	}
	tz_pwrlevels[0] = j;
	ret = scm_call(SCM_SVC_DCVS, TZ_INIT_ID, tz_pwrlevels,
				sizeof(tz_pwrlevels), NULL, 0);
	if (ret)
		priv->idle_dcvs = 1;
	return 0;
}
#else
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	return -EINVAL;
}
#endif /* CONFIG_MSM_SCM */

static void tz_close(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	kgsl_pwrscale_policy_remove_files(device, pwrscale, &tz_attr_group);
	kfree(pwrscale->priv);
	pwrscale->priv = NULL;
}

struct kgsl_pwrscale_policy kgsl_pwrscale_policy_tz = {
	.name = "trustzone",
	.init = tz_init,
	.busy = tz_busy,
	.idle = tz_idle,
	.sleep = tz_sleep,
	.wake = tz_wake,
	.close = tz_close
};
EXPORT_SYMBOL(kgsl_pwrscale_policy_tz);
