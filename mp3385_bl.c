/*
 * Based on linux/drivers/video/backlight/pwm_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/mutex.h>

/****************************************************************
* LED DRIVER SETTINGS                                           *
****************************************************************/
#define MP3385_WLED_DIMMING_MODE_REG    0x00
#define MP3385_WLED_OPFREQ_REG          0x01
#define MP3385_WLED_FULLSCALE_REG       0x02
#define MP3385_WLED_FAULT_REG           0x03
#define MP3385_WLED_DIMMING_BRIGHT_REG  0x04
#define MP3385_WLED_PROTECTION_THR_REG  0x05
#define MP3385_WLED_VENDOR_ID_REG       0x06


static int settings[7];
module_param_array(settings, int, NULL, S_IRUGO);
MODULE_PARM_DESC(settings, "MP3385 Settings");

struct mp3385_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	bool			enabled;
	struct regulator	*power_supply;
	struct gpio_desc	*enable_gpio;
	unsigned int		scale;
	bool			legacy;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
	char			fb_id[16];

	/* i2c client */
	struct regmap *regmap;
	struct i2c_client  *mp3385_dev;
    struct device_node *client_dev_node;
	struct mutex access_lock;
	int r_val_from_dts;
	int r_val_from_bootargs;
};


int mp3385_wled_dump(struct mp3385_bl_data *pb)
{
    int reg = 0;
    char wled_string[50];

    memset(wled_string, 0, 50);


    for (reg = 0; reg <= MP3385_WLED_VENDOR_ID_REG; reg++)
    {
       	unsigned int val = 0;
    	regmap_read(pb->regmap, reg, &val);
        sprintf(wled_string, "%s0x%02X ", wled_string, val);

    }
    pr_err("MP3385 WLED Settings %s", wled_string);

    return 0;
}

int mp3385_wled_init(struct mp3385_bl_data *pb)
{
    int reg = 0;

	/* write settings into register client-device */
    for (reg = 0; reg < MP3385_WLED_VENDOR_ID_REG; reg++)
      regmap_write(pb->regmap, reg, settings[reg]);

    return 0;
}

static void mp3385_backlight_power_on(struct mp3385_bl_data *pb, int brightness)
{
	int err;

	if (pb->enabled)
		return;

	err = regulator_enable(pb->power_supply);
	if (err < 0)
		dev_err(pb->dev, "failed to enable power supply\n");

	if (pb->enable_gpio)
		gpiod_set_value_cansleep(pb->enable_gpio, 1);

	/* Set register value before enable pwm */
	msleep(20);
    mp3385_wled_init(pb);
    mp3385_wled_dump(pb);

	pwm_enable(pb->pwm);
	pb->enabled = true;
}

static void mp3385_backlight_power_off(struct mp3385_bl_data *pb)
{
	if (!pb->enabled)
		return;

	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);

	if (pb->enable_gpio)
		gpiod_set_value_cansleep(pb->enable_gpio, 0);

	regulator_disable(pb->power_supply);
	pb->enabled = false;
}

static int mp3385_compute_duty_cycle(struct mp3385_bl_data *pb, int brightness)
{
	unsigned int lth = pb->lth_brightness;
	u64 duty_cycle;

	if (pb->levels)
		duty_cycle = pb->levels[brightness];
	else
		duty_cycle = brightness;

	duty_cycle *= pb->period - lth;
	do_div(duty_cycle, pb->scale);

	return duty_cycle + lth;
}

static int mp3385_backlight_update_status(struct backlight_device *bl)
{
	struct mp3385_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int duty_cycle;


	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness > 0) {

		duty_cycle = mp3385_compute_duty_cycle(pb, brightness);

		pwm_config(pb->pwm, duty_cycle, pb->period);

		mp3385_backlight_power_on(pb, brightness);

	} else
		mp3385_backlight_power_off(pb);

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int mp3385_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct mp3385_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops mp3385_backlight_ops = {
	.update_status	= mp3385_backlight_update_status,
	.check_fb	= mp3385_backlight_check_fb,
};

#ifdef CONFIG_OF
#define PWM_LUMINANCE_SCALE	10000 /* luminance scale */

/* An integer based power function */
static u64 int_pow(u64 base, int exp)
{
	u64 result = 1;

	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}

	return result;
}

/*
 * CIE lightness to PWM conversion.
 *
 * The CIE 1931 lightness formula is what actually describes how we perceive
 * light:
 *          Y = (L* / 902.3)           if L* ≤ 0.08856
 *          Y = ((L* + 16) / 116)^3    if L* > 0.08856
 *
 * Where Y is the luminance, the amount of light coming out of the screen, and
 * is a number between 0.0 and 1.0; and L* is the lightness, how bright a human
 * perceives the screen to be, and is a number between 0 and 100.
 *
 * The following function does the fixed point maths needed to implement the
 * above formula.
 */
static u64 cie1931(unsigned int lightness, unsigned int scale)
{
	u64 retval;

	lightness *= 100;
	if (lightness <= (8 * scale)) {
		retval = DIV_ROUND_CLOSEST_ULL(lightness * 10, 9023);
	} else {
		retval = int_pow((lightness + (16 * scale)) / 116, 3);
		retval = DIV_ROUND_CLOSEST_ULL(retval, (scale * scale));
	}

	return retval;
}

/*
 * Create a default correction table for PWM values to create linear brightness
 * for LED based backlights using the CIE1931 algorithm.
 */
static
int mp3385_pwm_backlight_brightness_default(struct device *dev,
				     struct platform_pwm_backlight_data *data,
				     unsigned int period)
{
	unsigned int counter = 0;
	unsigned int i, n;
	u64 retval;

	/*
	 * Count the number of bits needed to represent the period number. The
	 * number of bits is used to calculate the number of levels used for the
	 * brightness-levels table, the purpose of this calculation is have a
	 * pre-computed table with enough levels to get linear brightness
	 * perception. The period is divided by the number of bits so for a
	 * 8-bit PWM we have 255 / 8 = 32 brightness levels or for a 16-bit PWM
	 * we have 65535 / 16 = 4096 brightness levels.
	 *
	 * Note that this method is based on empirical testing on different
	 * devices with PWM of 8 and 16 bits of resolution.
	 */
	n = period;
	while (n) {
		counter += n % 2;
		n >>= 1;
	}

	data->max_brightness = DIV_ROUND_UP(period, counter);
	data->levels = devm_kcalloc(dev, data->max_brightness,
				    sizeof(*data->levels), GFP_KERNEL);
	if (!data->levels)
		return -ENOMEM;

	/* Fill the table using the cie1931 algorithm */
	for (i = 0; i < data->max_brightness; i++) {
		retval = cie1931((i * PWM_LUMINANCE_SCALE) /
				 data->max_brightness, PWM_LUMINANCE_SCALE) *
				 period;
		retval = DIV_ROUND_CLOSEST_ULL(retval, PWM_LUMINANCE_SCALE);
		if (retval > UINT_MAX)
			return -EINVAL;
		data->levels[i] = (unsigned int)retval;
	}

	data->dft_brightness = data->max_brightness / 2;
	data->max_brightness--;

	return 0;
}

static int mp3385_backlight_check_fb_name(struct device *dev, struct fb_info *info)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct mp3385_bl_data *pb = bl_get_data(bl);

	if (strcmp(info->fix.id, pb->fb_id) == 0)
		return true;

	return false;
}

static int mp3385_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	unsigned int num_levels = 0;
	unsigned int levels_count;
	unsigned int num_steps = 0;
	struct property *prop;
	unsigned int *table;
	int length;
	u32 value;
	int ret;
	const char *names;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	if (!of_property_read_string(node, "fb-names", &names)) {
		strcpy(data->fb_id, names);
		data->check_fb = &mp3385_backlight_check_fb_name;
	}

	/*
	 * These values are optional and set as 0 by default, the out values
	 * are modified only if a valid u32 value can be decoded.
	 */
	of_property_read_u32(node, "post-pwm-on-delay-ms",
			     &data->post_pwm_on_delay);
	of_property_read_u32(node, "pwm-off-delay-ms", &data->pwm_off_delay);

	data->enable_gpio = -EINVAL;

	/*
	 * Determine the number of brightness levels, if this property is not
	 * set a default table of brightness levels will be used.
	 */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return 0;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;
		unsigned int i, j, n = 0;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;

		data->dft_brightness = value;

		/*
		 * This property is optional, if is set enables linear
		 * interpolation between each of the values of brightness levels
		 * and creates a new pre-computed table.
		 */
		of_property_read_u32(node, "num-interpolated-steps",
				     &num_steps);

		/*
		 * Make sure that there is at least two entries in the
		 * brightness-levels table, otherwise we can't interpolate
		 * between two points.
		 */
		if (num_steps) {
			if (data->max_brightness < 2) {
				dev_err(dev, "can't interpolate\n");
				return -EINVAL;
			}

			/*
			 * Recalculate the number of brightness levels, now
			 * taking in consideration the number of interpolated
			 * steps between two levels.
			 */
			for (i = 0; i < data->max_brightness - 1; i++) {
				if ((data->levels[i + 1] - data->levels[i]) /
				   num_steps)
					num_levels += num_steps;
				else
					num_levels++;
			}
			num_levels++;
			dev_dbg(dev, "new number of brightness levels: %d\n",
				num_levels);

			/*
			 * Create a new table of brightness levels with all the
			 * interpolated steps.
			 */
			size = sizeof(*table) * num_levels;
			table = devm_kzalloc(dev, size, GFP_KERNEL);
			if (!table)
				return -ENOMEM;

			/* Fill the interpolated table. */
			levels_count = 0;
			for (i = 0; i < data->max_brightness - 1; i++) {
				value = data->levels[i];
				n = (data->levels[i + 1] - value) / num_steps;
				if (n > 0) {
					for (j = 0; j < num_steps; j++) {
						table[levels_count] = value;
						value += n;
						levels_count++;
					}
				} else {
					table[levels_count] = data->levels[i];
					levels_count++;
				}
			}
			table[levels_count] = data->levels[i];

			/*
			 * As we use interpolation lets remove current
			 * brightness levels table and replace for the
			 * new interpolated table.
			 */
			devm_kfree(dev, data->levels);
			data->levels = table;

			/*
			 * Reassign max_brightness value to the new total number
			 * of brightness levels.
			 */
			data->max_brightness = num_levels;
		}

		data->max_brightness--;
	}
	return 0;
}
#else
static int mp3385_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}

static
int mp3385_pwm_backlight_brightness_default(struct device *dev,
				     struct platform_pwm_backlight_data *data,
				     unsigned int period)
{
	return -ENODEV;
}
#endif

static int mp3385_backlight_initial_power_state(const struct mp3385_bl_data *pb)
{
	struct device_node *node = pb->dev->of_node;

	/* Not booted with device tree or no phandle link to the node */
	if (!node || !node->phandle)
		return FB_BLANK_UNBLANK;

	/*
	 * If the driver is probed from the device tree and there is a
	 * phandle link pointing to the backlight node, it is safe to
	 * assume that another driver will enable the backlight at the
	 * appropriate time. Therefore, if it is disabled, keep it so.
	 */

	/* if the enable GPIO is disabled, do not enable the backlight */
	if (pb->enable_gpio && gpiod_get_value_cansleep(pb->enable_gpio) == 0)
		return FB_BLANK_POWERDOWN;

	/* The regulator is disabled, do not enable the backlight */
	if (!regulator_is_enabled(pb->power_supply))
		return FB_BLANK_POWERDOWN;

	/* The PWM is disabled, keep it like this */
	if (!pwm_is_enabled(pb->pwm))
		return FB_BLANK_POWERDOWN;

	return FB_BLANK_UNBLANK;
}

static int mp3385_fetch_reg_val_from_dts(struct mp3385_bl_data *pb)
{
	u16 dts_val_i;
	int ret;

	/* populate settings from dts property reg-val */
	for (dts_val_i = 0; dts_val_i < ( MP3385_WLED_VENDOR_ID_REG + 1 ); dts_val_i++){
		ret = of_property_read_u32_index(pb->client_dev_node, "reg-val", dts_val_i, &settings[dts_val_i]);
		if(ret)
			return -EINVAL;
	}

	return ret;
}

static const struct regmap_config mps_mp3385_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MP3385_WLED_VENDOR_ID_REG,
};

static int mp3385_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = dev_get_platdata(&pdev->dev);
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct device_node *node = pdev->dev.of_node;
	struct mp3385_bl_data *pb;
	struct pwm_state state;
	struct pwm_args pargs;
	unsigned int i;
	int ret;
	u32 tmp;

	if (!data) {
		ret = mp3385_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}


	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}


	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* search for client-device node */
	pb->client_dev_node = of_parse_phandle(node, "client-device", 0);
	if (!pb->client_dev_node)
		dev_err(&pdev->dev, "client-device phandle missing or invalid\n");

	/* map client-device node to i2c_client struct */
	pb->mp3385_dev = of_find_i2c_device_by_node(pb->client_dev_node);
	if (!pb->mp3385_dev) {
		dev_err(&pdev->dev, "failed to find led-driver i2c node\n");
		return -EPROBE_DEFER;
	}

	/* if reg-val property list exist fetch register value from dts else from bootargs */
	if (of_get_property(pb->client_dev_node, "reg-val", &tmp)){
		ret = mp3385_fetch_reg_val_from_dts(pb);
	}

	/* store i2c client data struct value */
	i2c_set_clientdata(pb->mp3385_dev, pb);

	/* map i2c device register into the kernel */
	pb->regmap = devm_regmap_init_i2c(pb->mp3385_dev, &mps_mp3385_regmap_config);
	if (IS_ERR(pb->regmap)){
		return PTR_ERR(pb->regmap);
	}

	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;
	pb->enabled = false;
	strcpy(pb->fb_id, data->fb_id);

	pb->enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
						  GPIOD_ASIS);
	if (IS_ERR(pb->enable_gpio)) {
		ret = PTR_ERR(pb->enable_gpio);
		goto err_alloc;
	}

	/*
	 * Compatibility fallback for drivers still using the integer GPIO
	 * platform data. Must go away soon.
	 */
	if (!pb->enable_gpio && gpio_is_valid(data->enable_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, data->enable_gpio,
					    GPIOF_OUT_INIT_HIGH, "enable");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request GPIO#%d: %d\n",
				data->enable_gpio, ret);
			goto err_alloc;
		}

		pb->enable_gpio = gpio_to_desc(data->enable_gpio);
	}

	/*
	 * If the GPIO is not known to be already configured as output, that
	 * is, if gpiod_get_direction returns either 1 or -EINVAL, change the
	 * direction to output and set the GPIO as active.
	 * Do not force the GPIO to active when it was already output as it
	 * could cause backlight flickering or we would enable the backlight too
	 * early. Leave the decision of the initial backlight state for later.
	 */
	if (pb->enable_gpio &&
	    gpiod_get_direction(pb->enable_gpio) != 0)
		gpiod_direction_output(pb->enable_gpio, 1);

	pb->power_supply = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(pb->power_supply)) {
		ret = PTR_ERR(pb->power_supply);
		goto err_alloc;
	}

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm) && PTR_ERR(pb->pwm) != -EPROBE_DEFER && !node) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");
		pb->legacy = true;
		pb->pwm = pwm_request(data->pwm_id, "mp3385-backlight");
	}

	if (IS_ERR(pb->pwm)) {
		ret = PTR_ERR(pb->pwm);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "unable to request PWM\n");
		goto err_alloc;
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	if (!data->levels) {
		/* Get the PWM period (in nanoseconds) */
		pwm_get_state(pb->pwm, &state);

		ret = mp3385_pwm_backlight_brightness_default(&pdev->dev, data,
						       state.period);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to setup default brightness table\n");
			goto err_alloc;
		}
	}

	for (i = 0; i <= data->max_brightness; i++) {
		if (data->levels[i] > pb->scale)
			pb->scale = data->levels[i];

		pb->levels = data->levels;
	}

	/*
	 * FIXME: pwm_apply_args() should be removed when switching to
	 * the atomic PWM API.
	 */
	pwm_apply_args(pb->pwm);

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data if it has not already been set
	 * via the PWM lookup table.
	 */
	pwm_get_args(pb->pwm, &pargs);
	pb->period = pargs.period;
	if (!pb->period && (data->pwm_period_ns > 0))
		pb->period = data->pwm_period_ns;

	pb->lth_brightness = data->lth_brightness * (pb->period / pb->scale);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &mp3385_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		if (pb->legacy)
			pwm_free(pb->pwm);
		goto err_alloc;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	bl->props.power = mp3385_backlight_initial_power_state(pb);

	backlight_update_status(bl);
	platform_set_drvdata(pdev, bl);

	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int mp3385_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct mp3385_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	mp3385_backlight_power_off(pb);

	if (pb->exit)
		pb->exit(&pdev->dev);
	if (pb->legacy)
		pwm_free(pb->pwm);

	return 0;
}

static void mp3385_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct mp3385_bl_data *pb = bl_get_data(bl);

	mp3385_backlight_power_off(pb);
}

#ifdef CONFIG_PM_SLEEP
static int mp3385_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct mp3385_bl_data *pb = bl_get_data(bl);

	if (pb->notify)
		pb->notify(pb->dev, 0);

	mp3385_backlight_power_off(pb);

	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);

	return 0;
}

static int mp3385_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);

	return 0;
}
#endif

static const struct dev_pm_ops mp3385_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = mp3385_backlight_suspend,
	.resume = mp3385_backlight_resume,
	.poweroff = mp3385_backlight_suspend,
	.restore = mp3385_backlight_resume,
#endif
};

static const struct of_device_id mp3385_backlight_of_match[] = {
	{ .compatible = "mp3385-backlight" },
	{ .compatible = "seco,led_mp3385" },
	{ }
};

MODULE_DEVICE_TABLE(of, mp3385_backlight_of_match);

static struct platform_driver mp3385_backlight_driver = {
	.driver		= {
		.name		= "mp3385-backlight",
		.pm		= &mp3385_backlight_pm_ops,
		.of_match_table	= of_match_ptr(mp3385_backlight_of_match),
	},
	.probe		= mp3385_backlight_probe,
	.remove		= mp3385_backlight_remove,
	.shutdown	= mp3385_backlight_shutdown,
};

module_platform_driver(mp3385_backlight_driver);

MODULE_DESCRIPTION("MP3385 based Backlight Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michele Cirinei <michele.cirinei@seco.com> - Tommaso Merciai <tommaso.merciai@seco.com>");
