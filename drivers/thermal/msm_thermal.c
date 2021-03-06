/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal_ioctl.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <mach/cpufreq.h>
#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
#include "../../arch/arm/mach-msm/msm_mpdecision.h"
#endif

#define BYTES_PER_FUSE_ROW  8
#define MAX_EFUSE_VALUE  16

struct msm_thermal_ioctl_dev {
	struct semaphore sem;
	struct cdev char_dev;
};

static DEFINE_MUTEX(emergency_shutdown_mutex);
static uint32_t default_cpu_temp_limit;
static bool default_temp_limit_enabled;
static bool default_temp_limit_probed;
static bool default_temp_limit_nodes_called;

static int enabled;
static int msm_thermal_major;
static struct class *thermal_class;
static struct msm_thermal_ioctl_dev *msm_thermal_dev;

//Throttling indicator, 0=not throttled, 1=low, 2=mid, 3=max
int msm_thermal_x_throttled = 0;
EXPORT_SYMBOL_GPL(msm_thermal_x_throttled);

//Save the cpu max freq before throttling
static int pre_throttled_max = 0;

static struct msm_thermal_data msm_thermal_info;

enum efuse_data {
	EFUSE_ADDRESS = 0,
	EFUSE_SIZE,
	EFUSE_ROW,
	EFUSE_START_BIT,
	EFUSE_BIT_MASK,
	EFUSE_DATA_MAX,
};

static struct msm_thermal_stat msm_thermal_stats = {
    .time_low_start = 0,
    .time_mid_start = 0,
    .time_max_start = 0,
    .time_low = 0,
    .time_mid = 0,
    .time_max = 0,
};

static ssize_t default_cpu_temp_limit_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", default_cpu_temp_limit);
}

static struct delayed_work check_temp_work;
static struct workqueue_struct *check_temp_workq;

static void update_stats(void)
{
    if (msm_thermal_stats.time_low_start > 0) {
        msm_thermal_stats.time_low += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_low_start);
        msm_thermal_stats.time_low_start = 0;
    }
    if (msm_thermal_stats.time_mid_start > 0) {
        msm_thermal_stats.time_mid += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_mid_start);
        msm_thermal_stats.time_mid_start = 0;
    }
    if (msm_thermal_stats.time_max_start > 0) {
        msm_thermal_stats.time_max += (ktime_to_ms(ktime_get()) - msm_thermal_stats.time_max_start);
        msm_thermal_stats.time_max_start = 0;
    }
}

static void start_stats(int status)
{
    switch (msm_thermal_x_throttled) {
        case 1:
            msm_thermal_stats.time_low_start = ktime_to_ms(ktime_get());
            break;
        case 2:
            msm_thermal_stats.time_mid_start = ktime_to_ms(ktime_get());
            break;
        case 3:
            msm_thermal_stats.time_max_start = ktime_to_ms(ktime_get());
            break;
    }
}

static struct kobj_attribute default_cpu_temp_limit_attr =
		__ATTR_RO(default_cpu_temp_limit);

static int msm_thermal_add_default_temp_limit_nodes(void)
{
	struct kobject *module_kobj = NULL;
	int ret = 0;

	if (!default_temp_limit_probed) {
		default_temp_limit_nodes_called = true;
		return ret;
	}
	if (!default_temp_limit_enabled)
		return ret;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		return -ENOENT;
	}

	sysfs_attr_init(&default_cpu_temp_limit_attr.attr);
	ret = sysfs_create_file(module_kobj, &default_cpu_temp_limit_attr.attr);
	if (ret) {
		pr_err(
		"cannot create default_cpu_temp_limit attribute. err:%d\n",
		ret);
		return ret;
	}
	return ret;
}

static int update_cpu_max_freq(struct cpufreq_policy *cpu_policy,
                   int cpu, int max_freq)
{
    int ret = 0;

    if (!cpu_policy)
        return -EINVAL;

    cpufreq_verify_within_limits(cpu_policy, cpu_policy->min, max_freq);
    cpu_policy->user_policy.max = max_freq;

    ret = cpufreq_update_policy(cpu);
    if (!ret)
        pr_debug("msm_thermal: Setting CPU%d max frequency to %d\n",
                 cpu, max_freq);
    return ret;
}

static int get_efuse_temp_map(struct device_node *node,
				int *efuse_values,
				int *efuse_temp)
{
	uint32_t i, j, efuse_arr_cnt = 0;
	int ret = 0, efuse_map_cnt = 0;
	uint32_t data[2 * MAX_EFUSE_VALUE];

	char *key = "qcom,efuse-temperature-map";
	if (!of_get_property(node, key, &efuse_map_cnt)
		|| efuse_map_cnt <= 0) {
		pr_debug("Property %s not defined.\n", key);
		return -ENODEV;
	}

	if (efuse_map_cnt % (sizeof(__be32) * 2)) {
		pr_err("Invalid number(%d) of entry for %s\n",
				efuse_map_cnt, key);
		return -EINVAL;
	}

	efuse_arr_cnt = efuse_map_cnt / sizeof(__be32);

	ret = of_property_read_u32_array(node, key, data, efuse_arr_cnt);
	if (ret)
		return -EINVAL;

	efuse_map_cnt /= (sizeof(__be32) * 2);

	j = 0;
	for (i = 0; i < efuse_map_cnt; i++) {
		efuse_values[i] = data[j++];
		efuse_temp[i] = data[j++];
	}

	return efuse_map_cnt;
}

static int probe_thermal_efuse_read(struct device_node *node,
			struct msm_thermal_data *data,
			struct platform_device *pdev)
{
	u64 efuse_bits;
	int ret = 0;
	int i = 0;
	int efuse_map_cnt = 0;
	int efuse_data_cnt = 0;
	char *key = NULL;
	void __iomem *efuse_base = NULL;
	uint32_t efuse_data[EFUSE_DATA_MAX] = {0};
	uint32_t efuse_values[MAX_EFUSE_VALUE] = {0};
	uint32_t efuse_temp[MAX_EFUSE_VALUE] = {0};
	uint32_t default_temp = 0;
	uint8_t thermal_efuse_data = 0;

	if (default_temp_limit_probed)
		goto read_efuse_exit;

	key = "qcom,default-temp";
	if (of_property_read_u32(node, key, &default_temp))
		default_temp = 0;

	default_cpu_temp_limit = default_temp;

	key = "qcom,efuse-data";
	if (!of_get_property(node, key, &efuse_data_cnt) ||
		efuse_data_cnt <= 0) {
		ret = -ENODEV;
		goto read_efuse_fail;
	}
	efuse_data_cnt /= sizeof(__be32);

	if (efuse_data_cnt != EFUSE_DATA_MAX) {
		pr_err("Invalid number of efuse data. data cnt %d\n",
			efuse_data_cnt);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	ret = of_property_read_u32_array(node, key, efuse_data,
						efuse_data_cnt);
	if (ret)
		goto read_efuse_fail;

	if (efuse_data[EFUSE_ADDRESS] == 0 ||
		efuse_data[EFUSE_SIZE] == 0 ||
		efuse_data[EFUSE_BIT_MASK] == 0) {
		pr_err("Invalid efuse data: address:%x len:%d bitmask%x\n",
			efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_SIZE],
			efuse_data[EFUSE_BIT_MASK]);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_map_cnt = get_efuse_temp_map(node, efuse_values,
						efuse_temp);
	if (efuse_map_cnt <= 0 ||
		efuse_map_cnt > (efuse_data[EFUSE_BIT_MASK] + 1)) {
		pr_err("Invalid efuse-temperature-map. cnt%d\n",
			efuse_map_cnt);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_base = ioremap(efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_SIZE]);
	if (!efuse_base) {
		pr_err("Unable to map efuse_addr:%x with size%d\n",
			efuse_data[EFUSE_ADDRESS],
			efuse_data[EFUSE_SIZE]);
		ret = -EINVAL;
		goto read_efuse_fail;
	}

	efuse_bits = readll_relaxed(efuse_base
		+ efuse_data[EFUSE_ROW] * BYTES_PER_FUSE_ROW);

	thermal_efuse_data = (efuse_bits >> efuse_data[EFUSE_START_BIT]) &
						efuse_data[EFUSE_BIT_MASK];

	/* Get cpu limit temp from efuse truth table */
	for (; i < efuse_map_cnt; i++) {
		if (efuse_values[i] == thermal_efuse_data) {
			default_cpu_temp_limit = efuse_temp[i];
			break;
		}
	}
	if (i >= efuse_map_cnt) {
		if (!default_temp) {
			pr_err("No matching efuse value. value:%d\n",
				thermal_efuse_data);
			ret = -EINVAL;
			goto read_efuse_fail;
		}
	}

	pr_debug(
	"Efuse address:0x%x [row:%d] = 0x%llx @%d:mask:0x%x = 0x%x temp:%d\n",
		efuse_data[EFUSE_ADDRESS], efuse_data[EFUSE_ROW], efuse_bits,
		efuse_data[EFUSE_START_BIT], efuse_data[EFUSE_BIT_MASK],
		thermal_efuse_data, default_cpu_temp_limit);

	default_temp_limit_enabled = true;

read_efuse_fail:
	if (efuse_base)
		iounmap(efuse_base);
	default_temp_limit_probed = true;
	if (ret) {
		if (!default_temp) {
			dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
				__func__, node->full_name, key);
		} else {
			default_temp_limit_enabled = true;
			pr_debug("Default cpu temp limit is %d\n",
					default_cpu_temp_limit);
			ret = 0;
		}
	}
read_efuse_exit:
	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
 	int ret = 0;
 	struct device_node *node = pdev->dev.of_node;
 	struct msm_thermal_data data;

        probe_thermal_efuse_read(node, &data, pdev);
        if (default_temp_limit_nodes_called) {
		msm_thermal_add_default_temp_limit_nodes();
		default_temp_limit_nodes_called = false;
	}
        
        return ret;
}

#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
static int update_cpu_min_freq(struct cpufreq_policy *cpu_policy,
                               int cpu, int new_freq)
{
    int ret = 0;

    if (!cpu_policy)
        return -EINVAL;

    cpufreq_verify_within_limits(cpu_policy, new_freq, cpu_policy->max);
    cpu_policy->user_policy.min = new_freq;

    ret = cpufreq_update_policy(cpu);
    if (!ret) {
        pr_debug("msm_thermal: Setting CPU%d min frequency to %d\n",
            cpu, new_freq);
    }
    return ret;
}

DECLARE_PER_CPU(struct msm_mpdec_cpudata_t, msm_mpdec_cpudata);
#endif
static void check_temp(struct work_struct *work)
{
    struct cpufreq_policy *cpu_policy = NULL;
    struct tsens_device tsens_dev;
    unsigned long temp = 0;
    uint32_t max_freq = 0;
    bool update_policy = false;
    int i = 0, cpu = 0, ret = 0;

    tsens_dev.sensor_num = msm_thermal_info.sensor_id;
    ret = tsens_get_temp(&tsens_dev, &temp);
    if (ret) {
        pr_err("msm_thermal: FATAL: Unable to read TSENS sensor %d\n",
               tsens_dev.sensor_num);
        goto reschedule;
    }

    if (temp >= msm_thermal_info.shutdown_temp) {
        mutex_lock(&emergency_shutdown_mutex);
        pr_warn("################################\n");
        pr_warn("################################\n");
        pr_warn("- %u OVERTEMP! SHUTTING DOWN! -\n", msm_thermal_info.shutdown_temp);
        pr_warn("- cur temp:%lu measured by:%u -\n", temp, msm_thermal_info.sensor_id);
        pr_warn("################################\n");
        pr_warn("################################\n");
        /* orderly poweroff tries to power down gracefully
           if it fails it will force it. */
        orderly_poweroff(true);
        for_each_possible_cpu(cpu) {
            update_policy = true;
            max_freq = msm_thermal_info.allowed_max_freq;
            msm_thermal_x_throttled = 3;
            pr_warn("msm_thermal: Emergency throttled CPU%i to %u! temp:%lu\n",
                    cpu, msm_thermal_info.allowed_max_freq, temp);
        }
        mutex_unlock(&emergency_shutdown_mutex);
    }

    for_each_possible_cpu(cpu) {
        update_policy = false;
        cpu_policy = cpufreq_cpu_get(cpu);
        if (!cpu_policy) {
            pr_debug("msm_thermal: NULL policy on cpu %d\n", cpu);
            continue;
        }

        /* save pre-throttled max freq value */
        if ((msm_thermal_x_throttled == 0) && (cpu == 0))
            pre_throttled_max = cpu_policy->max;

        //low trip point
        if ((temp >= msm_thermal_info.allowed_low_high) &&
            (temp < msm_thermal_info.allowed_mid_high) &&
            (msm_thermal_x_throttled < 1)) {
            update_policy = true;
            max_freq = msm_thermal_info.allowed_low_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 1;
                pr_warn("msm_thermal: Thermal Throttled (low)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //low clr point
        } else if ((temp < msm_thermal_info.allowed_low_low) &&
               (msm_thermal_x_throttled > 0)) {
            if (pre_throttled_max != 0)
                max_freq = pre_throttled_max;
            else {
                max_freq = CONFIG_MSM_CPU_FREQ_MAX;
                pr_warn("msm_thermal: ERROR! pre_throttled_max=0, falling back to %u\n", max_freq);
            }
            update_policy = true;
            for (i = 1; i < CONFIG_NR_CPUS; i++) {
                if (cpu_online(i))
                        continue;
                cpu_up(i);
            }
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 0;
                pr_warn("msm_thermal: Low thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }

#ifdef CONFIG_MSM_MPDEC_INPUTBOOST_CPUMIN
            if (cpu_online(cpu)) {
                if (mutex_trylock(&per_cpu(msm_mpdec_cpudata, cpu).unboost_mutex)) {
                    per_cpu(msm_mpdec_cpudata, cpu).is_boosted = false;
                    update_cpu_min_freq(cpu_policy, cpu, per_cpu(msm_mpdec_cpudata, cpu).norm_min_freq);
                    mutex_unlock(&per_cpu(msm_mpdec_cpudata, cpu).unboost_mutex);
                }
            }
#endif

        //mid trip point
        } else if ((temp >= msm_thermal_info.allowed_mid_high) &&
               (temp < msm_thermal_info.allowed_max_high) &&
               (msm_thermal_x_throttled < 2)) {
            update_policy = true;
            max_freq = msm_thermal_info.allowed_mid_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 2;
                pr_warn("msm_thermal: Thermal Throttled (mid)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //mid clr point
        } else if ((temp < msm_thermal_info.allowed_mid_low) &&
               (msm_thermal_x_throttled > 1)) {
            max_freq = msm_thermal_info.allowed_low_freq;
            update_policy = true;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 1;
                pr_warn("msm_thermal: Mid thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //max trip point
        } else if (temp >= msm_thermal_info.allowed_max_high) {
            update_policy = true;
            max_freq = msm_thermal_info.allowed_max_freq;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 3;
                pr_warn("msm_thermal: Thermal Throttled (max)! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        //max clr point
        } else if ((temp < msm_thermal_info.allowed_max_low) &&
               (msm_thermal_x_throttled > 2)) {
            max_freq = msm_thermal_info.allowed_mid_freq;
            update_policy = true;
            if (cpu == (CONFIG_NR_CPUS-1)) {
                msm_thermal_x_throttled = 2;
                pr_warn("msm_thermal: Max thermal throttle ended! temp:%lu by:%u\n",
                        temp, msm_thermal_info.sensor_id);
            }
        }
        update_stats();
        start_stats(msm_thermal_x_throttled);
        if (update_policy)
            update_cpu_max_freq(cpu_policy, cpu, max_freq);

        cpufreq_cpu_put(cpu_policy);
    }

reschedule:
    if (enabled)
        queue_delayed_work(check_temp_workq, &check_temp_work,
                           msecs_to_jiffies(msm_thermal_info.poll_ms));

    return;
}

static void disable_msm_thermal(void)
{
    int cpu = 0;
    struct cpufreq_policy *cpu_policy = NULL;

     enabled = 0;
    /* make sure check_temp is no longer running */
    cancel_delayed_work_sync(&check_temp_work);

    if (pre_throttled_max != 0) {
        for_each_possible_cpu(cpu) {
            cpu_policy = cpufreq_cpu_get(cpu);
            if (cpu_policy) {
                if (cpu_policy->max < cpu_policy->cpuinfo.max_freq)
                    update_cpu_max_freq(cpu_policy, cpu, pre_throttled_max);
                cpufreq_cpu_put(cpu_policy);
            }
        }
    }

   pr_warn("msm_thermal: Warning! Thermal guard disabled!");
}

static void enable_msm_thermal(void)
{
    enabled = 1;
    /* make sure check_temp is running */
    queue_delayed_work(check_temp_workq, &check_temp_work,
                       msecs_to_jiffies(msm_thermal_info.poll_ms));

    pr_info("msm_thermal: Thermal guard enabled.");
}

static int set_enabled(const char *val, const struct kernel_param *kp)
{
    int ret = 0;

    ret = param_set_bool(val, kp);
    if (!enabled)
        disable_msm_thermal();
    else if (enabled == 1)
        enable_msm_thermal();
    else
        pr_info("msm_thermal: no action for enabled = %d\n", enabled);

    pr_info("msm_thermal: enabled = %d\n", enabled);

    return ret;
}

void msm_thermal_ioctl_cleanup()
{
	dev_t thermal_dev = MKDEV(msm_thermal_major, 0);

	if (!msm_thermal_dev) {
		pr_err("%s: Thermal IOCTL cleanup already done\n",
			KBUILD_MODNAME);
		return;
	}

	device_destroy(thermal_class, thermal_dev);
	class_destroy(thermal_class);
	cdev_del(&msm_thermal_dev->char_dev);
	unregister_chrdev_region(thermal_dev, 1);
	kfree(msm_thermal_dev);
	msm_thermal_dev = NULL;
	thermal_class = NULL;
}

static struct kernel_param_ops module_ops = {
    .set = set_enabled,
    .get = param_get_bool,
};

module_param_cb(enabled, &module_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "enforce thermal limit on cpu");

/**************************** SYSFS START ****************************/
struct kobject *msm_thermal_kobject;

#define show_one(file_name, object)                             \
static ssize_t show_##file_name                                 \
(struct kobject *kobj, struct attribute *attr, char *buf)       \
{                                                               \
    return sprintf(buf, "%u\n", msm_thermal_info.object);       \
}

show_one(shutdown_temp, shutdown_temp);
show_one(allowed_max_high, allowed_max_high);
show_one(allowed_max_low, allowed_max_low);
show_one(allowed_max_freq, allowed_max_freq);
show_one(allowed_mid_high, allowed_mid_high);
show_one(allowed_mid_low, allowed_mid_low);
show_one(allowed_mid_freq, allowed_mid_freq);
show_one(allowed_low_high, allowed_low_high);
show_one(allowed_low_low, allowed_low_low);
show_one(allowed_low_freq, allowed_low_freq);
show_one(poll_ms, poll_ms);

static ssize_t store_shutdown_temp(struct kobject *a, struct attribute *b,
                                   const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.shutdown_temp = input;

    return count;
}

static ssize_t store_allowed_max_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_high = input;

    return count;
}

static ssize_t store_allowed_max_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_low = input;

    return count;
}

static ssize_t store_allowed_max_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_max_freq = input;

    return count;
}

static ssize_t store_allowed_mid_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_high = input;

    return count;
}

static ssize_t store_allowed_mid_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_low = input;

    return count;
}

static ssize_t store_allowed_mid_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_mid_freq = input;

    return count;
}

static ssize_t store_allowed_low_high(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_high = input;

    return count;
}

static ssize_t store_allowed_low_low(struct kobject *a, struct attribute *b,
                                     const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_low = input;

    return count;
}

static ssize_t store_allowed_low_freq(struct kobject *a, struct attribute *b,
                                      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.allowed_low_freq = input;

    return count;
}

static ssize_t store_poll_ms(struct kobject *a, struct attribute *b,
                             const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
        return -EINVAL;

    msm_thermal_info.poll_ms = input;

    return count;
}

define_one_global_rw(shutdown_temp);
define_one_global_rw(allowed_max_high);
define_one_global_rw(allowed_max_low);
define_one_global_rw(allowed_max_freq);
define_one_global_rw(allowed_mid_high);
define_one_global_rw(allowed_mid_low);
define_one_global_rw(allowed_mid_freq);
define_one_global_rw(allowed_low_high);
define_one_global_rw(allowed_low_low);
define_one_global_rw(allowed_low_freq);
define_one_global_rw(poll_ms);

static struct attribute *msm_thermal_attributes[] = {
    &shutdown_temp.attr,
    &allowed_max_high.attr,
    &allowed_max_low.attr,
    &allowed_max_freq.attr,
    &allowed_mid_high.attr,
    &allowed_mid_low.attr,
    &allowed_mid_freq.attr,
    &allowed_low_high.attr,
    &allowed_low_low.attr,
    &allowed_low_freq.attr,
    &poll_ms.attr,
    NULL
};


static struct attribute_group msm_thermal_attr_group = {
    .attrs = msm_thermal_attributes,
    .name = "conf",
};

/********* STATS START *********/

static ssize_t show_throttle_times(struct kobject *a, struct attribute *b,
                                 char *buf)
{
    ssize_t len = 0;

    if (msm_thermal_x_throttled == 1) {
        len += sprintf(buf + len, "%s %llu\n", "low",
                       (msm_thermal_stats.time_low +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_low_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "low", msm_thermal_stats.time_low);

    if (msm_thermal_x_throttled == 2) {
        len += sprintf(buf + len, "%s %llu\n", "mid",
                       (msm_thermal_stats.time_mid +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_mid_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "mid", msm_thermal_stats.time_mid);

    if (msm_thermal_x_throttled == 3) {
        len += sprintf(buf + len, "%s %llu\n", "max",
                       (msm_thermal_stats.time_max +
                        (ktime_to_ms(ktime_get()) -
                         msm_thermal_stats.time_max_start)));
    } else
        len += sprintf(buf + len, "%s %llu\n", "max", msm_thermal_stats.time_max);

    return len;
}
define_one_global_ro(throttle_times);

static ssize_t show_is_throttled(struct kobject *a, struct attribute *b,
                                 char *buf)
{
    return sprintf(buf, "%u\n", msm_thermal_x_throttled);
}
define_one_global_ro(is_throttled);

static struct attribute *msm_thermal_stats_attributes[] = {
    &is_throttled.attr,
    &throttle_times.attr,
    NULL
};


static struct attribute_group msm_thermal_stats_attr_group = {
    .attrs = msm_thermal_stats_attributes,
    .name = "stats",
};
/**************************** SYSFS END ****************************/

static int msm_thermal_dev_exit(struct platform_device *inp_dev)
{
	msm_thermal_ioctl_cleanup();
	return 0;
}

static struct of_device_id msm_thermal_match_table[] = {
 	{.compatible = "qcom,msm-thermal"},
 	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
	.remove = msm_thermal_dev_exit,
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

int __init msm_thermal_init(struct msm_thermal_data *pdata)
{
    int ret = 0, rc = 0;

    BUG_ON(!pdata);
    BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
    memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

    enabled = 1;
    check_temp_workq=alloc_workqueue("msm_thermal", WQ_UNBOUND | WQ_RESCUER, 1);
    if (!check_temp_workq)
        BUG_ON(ENOMEM);
    INIT_DELAYED_WORK(&check_temp_work, check_temp);
    queue_delayed_work(check_temp_workq, &check_temp_work, 0);

    msm_thermal_kobject = kobject_create_and_add("msm_thermal", kernel_kobj);
    if (msm_thermal_kobject) {
        rc = sysfs_create_group(msm_thermal_kobject, &msm_thermal_attr_group);
        if (rc) {
            pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs group");
        }
        rc = sysfs_create_group(msm_thermal_kobject,
                                &msm_thermal_stats_attr_group);
        if (rc) {
            pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs stats group");
        }
    } else
        pr_warn("msm_thermal: sysfs: ERROR, could not create sysfs kobj");

    pr_info("%s complete.", __func__);

    return ret;
}

int __init msm_thermal_late_init(void)
{
       msm_thermal_add_default_temp_limit_nodes();

       return 0;
}
late_initcall(msm_thermal_late_init);
