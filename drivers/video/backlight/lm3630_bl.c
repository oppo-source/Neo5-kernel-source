#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/platform_data/lm3630_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <mach/gpio.h>
#include <mach/oppo_boot_mode.h>
#include <mach/device_info.h>
#include <mach/oppo_project.h>

#define REG_CTRL	0x00
#define REG_CONFIG	0x01
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_MAX		0x1F
#define CABC_DISABLE_LEVEL 0x28
#define REG_REVISION 0x1F


#define REG_MAXCU_A	0x05
#define REG_MAXCU_B	0x06


#define INT_DEBOUNCE_MSEC	10

void lm3630_control(int bl_level);
static struct lm3630_chip_data *lm3630_pchip;

static bool pwm_disabled= true;
int set_backlight_pwm(int state);
extern int cabc_mode;
static int backlight_level;
static bool pwm_flag = true;

struct lm3630_chip_data {
	struct i2c_client *client;
	struct device *dev;
	struct regulator *vcc_i2c;
	struct lm3630_platform_data *pdata;
	struct backlight_device *bled1;
	struct backlight_device *bled2;
};

enum lm3630_leds {
	BLED_ALL = 0,
	BLED_1,
	BLED_2
};

static const char * const bled_name[] = {
	[BLED_ALL] = "lm3630_bled",	/*Bank1 controls all string */
	[BLED_1] = "lm3630_bled1",	/*Bank1 controls bled1 */
	[BLED_2] = "lm3630_bled2",	/*Bank1 or 2 controls bled2 */
};

static int lm3630_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int lm3630_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int lm3630_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return lm3630_i2c_write(client, buf, sizeof(buf));
}

static int lm3630_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return lm3630_i2c_read(client, &addr, 1, val, 1);
}

static void lm3630_backlight_unregister(struct lm3630_chip_data *pchip)
{
	if (pchip->bled1)
		backlight_device_unregister(pchip->bled1);
	if (pchip->bled2)
		backlight_device_unregister(pchip->bled2);
}


static bool set_intensity(struct backlight_device *bl, struct lm3630_chip_data *pchip)
{
	if (!pchip->pdata->pwm_set_intensity)
		return false;
	
	pchip->pdata->pwm_set_intensity(bl->props.brightness - 1,
					pchip->pdata->pwm_period);
	return true;
}


/* update and get brightness */
/*
static int lm3630_bank_a_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	// brightness 0 means disable 
	if (!bl->props.brightness) {
		
		ret = lm3630_write_reg(pchip->client,REG_CTRL,0x80);
		if (ret < 0)
			goto out;
		return bl->props.brightness;
	}

	// pwm control 
	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev, "No pwm control func. in plat-data\n");
	} else {
		u8 val;

		// i2c control 
		ret = lm3630_read_reg(pchip->client,REG_CTRL,&val);
		val = val&0x7F;
		ret = lm3630_write_reg(pchip->client,REG_CTRL,val);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = lm3630_write_reg(pchip->client,REG_BRT_A,bl->props.brightness - 1);
		if (ret < 0)
			goto out;
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access REG_CTRL\n");
	return bl->props.brightness;
}

static int lm3630_bank_a_get_brightness(struct backlight_device *bl)
{
	u8 reg_val;	
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = lm3630_read_reg(pchip->client,REG_PWM_OUTHIGH,&reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = lm3630_read_reg(pchip->client,REG_PWM_OUTLOW,&reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
	} else {
		ret = lm3630_read_reg(pchip->client,REG_CTRL,&reg_val);
		reg_val = reg_val&0x7F;
		ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = lm3630_read_reg(pchip->client,REG_BRT_A,&reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
	}
	bl->props.brightness = brightness;
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}


static const struct backlight_ops lm3630_bank_a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	//.update_status = lm3630_bank_a_update_status,
	.get_brightness = lm3630_bank_a_get_brightness,
};
*/
static int lm3630_bank_b_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev,
				"no pwm control func. in plat-data\n");
	} else {
		u8 reg_val;
		ret = lm3630_read_reg(pchip->client,REG_CTRL,&reg_val);
		reg_val = reg_val&0x7F;
		ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = lm3630_write_reg(pchip->client,REG_BRT_B,bl->props.brightness - 1);
		if (ret < 0)
			goto out;
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return bl->props.brightness;
}

static int lm3630_bank_b_get_brightness(struct backlight_device *bl)
{
	u8 reg_val;
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = lm3630_read_reg(pchip->client,REG_PWM_OUTHIGH,&reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = lm3630_read_reg(pchip->client,REG_PWM_OUTLOW,&reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
		
	} else {

		u8 reg_val;
		ret = lm3630_read_reg(pchip->client,REG_CTRL,&reg_val);
		reg_val = reg_val&0x7F;
		ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = lm3630_read_reg(pchip->client,REG_BRT_B,&reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
		
	}
	bl->props.brightness = brightness;

	return bl->props.brightness;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return bl->props.brightness;
}


static const struct backlight_ops lm3630_bank_b_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630_bank_b_update_status,
	.get_brightness = lm3630_bank_b_get_brightness,
};



static int lm3630_backlight_register(struct lm3630_chip_data *pchip,
				     enum lm3630_leds ledno)
{
	const char *name = bled_name[ledno];
	struct backlight_properties props;
	struct lm3630_platform_data *pdata = pchip->pdata;

	props.type = BACKLIGHT_RAW;
	switch (ledno) {
	case BLED_1:
	case BLED_ALL:
		props.brightness = pdata->init_brt_led1;
		props.max_brightness = pdata->max_brt_led1;
		pchip->bled1 =
		    backlight_device_register(name, pchip->dev, pchip,
			//		      &lm3630_bank_a_ops, &props);
						  NULL, &props);
		if (IS_ERR(pchip->bled1))
			return PTR_ERR(pchip->bled1);
		break;
	case BLED_2:
		props.brightness = pdata->init_brt_led2;
		props.max_brightness = pdata->max_brt_led2;
		pchip->bled2 =
		    backlight_device_register(name, pchip->dev, pchip,
					      &lm3630_bank_b_ops, &props);
		if (IS_ERR(pchip->bled2))
			return PTR_ERR(pchip->bled2);
		break;
	}
	return 0;
}


static int lm3630_dt(struct device *dev, struct lm3630_platform_data *pdata)
{
	u32 temp_val;
	int rc;
	struct device_node *np = dev->of_node;
		
	rc = of_property_read_u32(np, "ti,bank-a-ctrl", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read bank-a-ctrl\n");
		pdata->bank_a_ctrl=BANK_A_CTRL_ALL;
	} else{
		pdata->bank_a_ctrl=temp_val;
	}
	
	rc = of_property_read_u32(np, "ti,init-brt-ed1", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to readinit-brt-ed1\n");
		pdata->init_brt_led1=200;
	} else{
		pdata->init_brt_led1=temp_val;
	}
	
	rc = of_property_read_u32(np, "ti,init-brt-led2", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read init-brt-led2\n");
		pdata->init_brt_led2=200;
	} else{
		pdata->init_brt_led2=temp_val;
	}
	
	rc = of_property_read_u32(np, "ti,max-brt-led1", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read max-brt-led1\n");
		pdata->max_brt_led1=255;
	} else{
		pdata->max_brt_led1=temp_val;
	}
	
	rc = of_property_read_u32(np, "ti,max-brt-led2", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read max-brt-led2\n");
		pdata->max_brt_led2=255;
	} else{
		pdata->max_brt_led2=temp_val;
	}
	
	rc = of_property_read_u32(np, "ti,pwm-active", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read pwm-active\n");
		pdata->pwm_active=PWM_ACTIVE_HIGH;
	} else{
		pdata->pwm_active=temp_val;
	}
	if(get_boot_mode() == MSM_BOOT_MODE__FACTORY) {
		pdata->pwm_ctrl=PWM_CTRL_DISABLE;
	}else{
		rc = of_property_read_u32(np, "ti,pwm-ctrl", &temp_val);
		if (rc) {
			dev_err(dev, "Unable to read pwm-ctrl\n");
			pdata->pwm_ctrl=PWM_CTRL_DISABLE;
			pwm_disabled = true;
		}else{
			pdata->pwm_ctrl=temp_val;
			pwm_disabled = false;
		}
		if(is_project(OPPO_13095)){
			if(get_PCB_Version() < HW_VERSION__12)
				pdata->pwm_ctrl=PWM_CTRL_DISABLE;	
		}
	}
	
	rc = of_property_read_u32(np, "ti,pwm-period", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read pwm-period\n");
		pdata->pwm_period=255;
	} else{
		pdata->pwm_period=temp_val;
	}	
	pdata->enable_gpio = of_get_named_gpio(np, "ti,enable-gpio",0);
	if(!pdata->enable_gpio){
		pr_err("pdata->enable_gpio is not specified");	
	}
	return 0;
}


static int lm3630_chip_init(struct lm3630_chip_data *pchip)
{
	int ret;
	u8 reg_val;
	u8 val;
	struct lm3630_platform_data *pdata = pchip->pdata;	
	
	/*pwm control */
	if(pdata->pwm_ctrl==PWM_CTRL_DISABLE)
		reg_val=0x18;
	else
		reg_val=0x19;
	ret = lm3630_write_reg(pchip->client,REG_CONFIG,reg_val);
	if (ret < 0){
		pr_err("lm3630 write REG_CONFIG failed\n");
		goto out;
	}
	//config max A and B current to 20mA
	ret = lm3630_write_reg(pchip->client,REG_MAXCU_A,0x14);
	ret = lm3630_write_reg(pchip->client,REG_MAXCU_B,0x14);

	/* bank control */
	reg_val = ((pdata->bank_b_ctrl & 0x01) << 1) |
			(pdata->bank_a_ctrl & 0x07)|0x18;//linear
	ret = lm3630_read_reg(pchip->client,REG_CTRL,&val);
	if (ret < 0)
		goto out;
	reg_val |= val&0x78;
	printk("LM3630:reg_val REG_CTRL 0x00 is %x\n",reg_val);
	ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
	if (ret < 0)
		goto out;
		
	/* set initial brightness */
	if (pdata->bank_a_ctrl != BANK_A_CTRL_DISABLE) {
		
		ret = lm3630_write_reg(pchip->client,REG_BRT_A,pdata->init_brt_led1);
		if (ret < 0)
			goto out;
		printk("%s: bl_initvalue=%d,222\n", __func__,pdata->init_brt_led1);
		if (ret < 0)
			goto out;
	}

	if (pdata->bank_b_ctrl != BANK_B_CTRL_DISABLE) {
		ret = lm3630_write_reg(pchip->client,REG_BRT_B,pdata->init_brt_led2);
		if (ret < 0)
			goto out;
	}
	return ret;
out:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return ret;
}

static ssize_t ftmbacklight_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    int level;
    if (!count)
		return -EINVAL;
/* OPPO 2014-05-30 yxq modify begin for all modes */
	pr_err("%s boot_mode is %d\n", __func__, get_boot_mode()); 
	level = simple_strtoul(buf, NULL, 10);
   	lm3630_control(level);
/* OPPO 2014-05-30 yxq modify end */
    
    return count;
}
    DEVICE_ATTR(ftmbacklight, 0644, NULL, ftmbacklight_store);


static int lm3630_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct lm3630_platform_data *pdata = client->dev.platform_data;
	struct lm3630_chip_data *pchip;
	int ret;
	static char *temp;
	unsigned char revision;
	printk("%s:is called.\n", __func__);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct lm3630_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = lm3630_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = client->dev.platform_data;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630_chip_data),
			     GFP_KERNEL);
	if (!pchip){
		printk("kzalloc pchip failed\n");
		return -ENOMEM;		
	}
	lm3630_pchip=pchip;
	
	pchip->pdata = pdata;
	pchip->dev = &client->dev;
	pchip->client = client;

	pchip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(pchip->vcc_i2c)) {
		ret = PTR_ERR(pchip->vcc_i2c);
		dev_err(&client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", ret);
		return -1;
	}	if (regulator_count_voltages(pchip->vcc_i2c) > 0) {
		ret = regulator_set_voltage(pchip->vcc_i2c, 1800000,
					   1800000);
		if (ret) {
			dev_err(&client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", ret);
			goto reg_vcc_i2c_put;		}	}

	ret = regulator_enable(pchip->vcc_i2c);
	if (ret) {
		dev_err(&client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", ret);
	}
	
//HW enable 
	ret = gpio_request(pchip->pdata->enable_gpio, "lm3630_enable");
	if (ret) {
		pr_err("lm3528_enable gpio_request failed: %d\n", ret);
		goto err_gpio_req;
	}
	ret = gpio_direction_output(pchip->pdata->enable_gpio, 1);
	if (ret) {
		pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
		goto err_gpio_req;
	}

	//pr_err("%s: gpio23 value = %d\n", __func__,gpio_get_value(23));

//HW enable
	i2c_set_clientdata(client, pchip);
	dev_set_drvdata(&client->dev,pchip);

	//just test lcd backlight
	/*
	lm3630_write_reg(pchip->client,0x01, 0x18);
	lm3630_write_reg(pchip->client,0x00, 0x05);
	lm3630_write_reg(pchip->client,0x05, 0x14);
	lm3630_write_reg(pchip->client,0x06, 0x14);
	lm3630_write_reg(pchip->client,0x03, 0xc8);//set backlight to 200
	lm3630_write_reg(pchip->client,0x04, 0xc8);*/

	/* chip initialize */
	msleep(500);
	ret = lm3630_chip_init(pchip);
	if (ret < 0) {
		msleep(300);
		ret = lm3630_chip_init(pchip);	
		if (ret < 0) {	
			dev_err(&client->dev, "fail : init chip\n");
			goto err_chip_init;
		}
	}
	
	lm3630_read_reg(client,REG_REVISION,&revision);
    if (revision == 0x02) {
        temp = "02";
   	} else {
        temp = "unknown";
    }
   	register_device_proc("backlight", temp, "LM3630A");
	
	switch (pdata->bank_a_ctrl) {
	case BANK_A_CTRL_ALL:
		printk("%s :BANK_A_CTRL_ALL",__func__);
		ret = lm3630_backlight_register(pchip, BLED_ALL);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	case BANK_A_CTRL_LED1:
		printk("%s :BANK_A_CTRL_LED1",__func__);
		ret = lm3630_backlight_register(pchip, BLED_1);
		break;
	case BANK_A_CTRL_LED2:
		printk("%s :BANK_A_CTRL_LED2",__func__);
		ret = lm3630_backlight_register(pchip, BLED_2);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	default:
		break;
	}

	if (ret < 0)
		goto err_bl_reg;

	if (pdata->bank_b_ctrl && pchip->bled2 == NULL) {
		ret = lm3630_backlight_register(pchip, BLED_2);
		if (ret < 0)
			goto err_bl_reg;
	}


    ret = device_create_file(&client->dev, &dev_attr_ftmbacklight);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create node ftmbacklight\n");
	}

	return 0;
err_bl_reg:
	dev_err(&client->dev, "fail : backlight register.\n");
	lm3630_backlight_unregister(pchip);	
	
err_chip_init:
	devm_kfree(&client->dev,pchip);
	devm_kfree(&client->dev,pdata);
	lm3630_pchip = NULL;
	
err_gpio_req:
	if (gpio_is_valid(pchip->pdata->enable_gpio))
		gpio_free(pchip->pdata->enable_gpio);
		
	
reg_vcc_i2c_put:
	regulator_put(pchip->vcc_i2c);

	return ret;
	
}


static int lm3630_remove(struct i2c_client *client)
{
	int ret;
	struct lm3630_chip_data *pchip = i2c_get_clientdata(client);
    device_remove_file(&client->dev, &dev_attr_ftmbacklight);
	ret = lm3630_write_reg(pchip->client,REG_BRT_A,0);
	if (ret < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	ret = lm3630_read_reg(pchip->client, REG_BRT_B, 0);
	if (ret < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	lm3630_backlight_unregister(pchip);
	
	if (gpio_is_valid(pchip->pdata->enable_gpio))
		gpio_free(pchip->pdata->enable_gpio);
	
	return 0;

}


static const struct i2c_device_id lm3630_id[] = {
	{LM3630_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3630_id);

static int lm3630_suspend(struct device *dev)
{
	int ret ;
	u8 reg_val;
	struct lm3630_chip_data *pchip = dev_get_drvdata(dev);
	printk("%s:backlight suspend.\n", __func__);
	
	ret = lm3630_read_reg(pchip->client,REG_CTRL,&reg_val);
	reg_val = reg_val|0x80;
	ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
	if (ret  < 0)
	{
		pr_err("%s: unable to shotdown !!!!!!!!!!!!\n", __func__);
	}

	return 0;
}


static int lm3630_resume(struct device *dev)
{
	int ret ;
	u8 reg_val;
	struct lm3630_chip_data *pchip = dev_get_drvdata(dev);

	printk("%s: backlight resume.\n", __func__);
	
	ret = lm3630_read_reg(pchip->client,REG_CTRL,&reg_val);
	reg_val = reg_val&0x7F;
	ret = lm3630_write_reg(pchip->client,REG_CTRL,reg_val);
	if (ret  < 0)
	{
		pr_err("%s: unable to shotdown !!!!!!!!!!!!\n", __func__);
	}	

	return 0;
}


static struct of_device_id lm3630_table[] = {
	{ .compatible = "ti,lm3630bl",},
	{ },
};

static const struct dev_pm_ops lm3630_pm_ops = {
	.resume		= lm3630_resume,
	.suspend	= lm3630_suspend,
};


static struct i2c_driver lm3630_i2c_driver = {
	.driver = {
		  .name 			= LM3630_NAME,
		  .owner			= THIS_MODULE,
		  .of_match_table 	= lm3630_table,	
	//	  .pm				= &lm3630_pm_ops,
		  },
	.probe = lm3630_probe,
	.remove = lm3630_remove,
	.id_table = lm3630_id,
};

int is_suspend = 0;
int set_backlight_pwm(int state)
{
	int ret;
	struct lm3630_chip_data *pchip = lm3630_pchip;
	if(is_project(OPPO_13095)){
		if((get_PCB_Version() < HW_VERSION__12)){
			ret=lm3630_write_reg(pchip->client,0x01,0x18);
			return 0;
		//	pr_err("LM3630 set pwm__________0x14 flag true\n");
		}
	}
	
	if((state==1)&&(backlight_level <= CABC_DISABLE_LEVEL)){	
	//	pr_err("LM3630 set pwm__________0x14 flag true\n");
			return 0;
	}else if(state == 1){
		ret=lm3630_write_reg(pchip->client,0x01,0x19);
		pwm_flag = true;
	//	pr_err("LM3630 set pwm__________flag true\n");
	}else{ 
		ret=lm3630_write_reg(pchip->client,0x01,0x18);
		pwm_flag = false;
	//	pr_err("LM3630 set pwm__________flag false\n");
	}
	return ret;
}

int set_proper_brightness(int bl_level)
{
	if(is_project(OPPO_13095)){
		if(get_PCB_Version() < HW_VERSION__12)
			return bl_level;
	}
	
	if((bl_level<=CABC_DISABLE_LEVEL)&&(bl_level>(CABC_DISABLE_LEVEL-10)))
	    bl_level = CABC_DISABLE_LEVEL-10;	
	return bl_level;
}

void lm3630_control(int bl_level)
{
	int ret;
	struct lm3630_chip_data *pchip = lm3630_pchip;
	if (!pchip){
		pr_err("LM3630:pchip is NULL should nerver call this\n");
		return;
	}
//	printk("%s:name = %s pid = %d\n",__func__,current->comm,current->pid);
	//dump_stack();
//	printk("%s: bl=%d\n", __func__,bl_level);
	if(bl_level>255) {
		bl_level = 255;
	}else if(bl_level == 0) {
		printk("LM3630 should set backlight to 0\n");
		ret = lm3630_write_reg(pchip->client,0x00,0xC0);
		if(ret<0)
			ret = lm3630_write_reg(pchip->client,0x00,0xC0);
		is_suspend = 1;
		return;
	}
	backlight_level=bl_level;
	bl_level=set_proper_brightness(bl_level);
	if(is_suspend == 1) {	
		lm3630_write_reg(pchip->client,0x00,0x1F);
		msleep(2);
		lm3630_write_reg(pchip->client,REG_BRT_A,bl_level);
		msleep(2);
		lm3630_write_reg(pchip->client,0x05,0x14);
		is_suspend = 0;		
	} else {
		ret = lm3630_write_reg(pchip->client,REG_BRT_A,bl_level);	
		is_suspend = 0;
	} 
/* OPPO 2014-06-11 gousj Modify begin for 14017 backlight not work */
#ifdef VENDOR_EDIT
	if((is_project(OPPO_13095))||(is_project(OPPO_14029))){ 
#endif
/* OPPO 2014-06-11 gousj Modify end */
		if(bl_level <= CABC_DISABLE_LEVEL && pwm_flag==true){
			set_backlight_pwm(0);
		//	pr_err("lm3630_control set_backlight_pwm 0");
		}else if(bl_level > CABC_DISABLE_LEVEL && pwm_flag==false && cabc_mode > 0){
			set_backlight_pwm(1);
		//	pr_err("lm3630_control set_backlight_pwm 1");
		}
/* OPPO 2014-05-30 yxq add begin for 14029 ftm mode backlight flick */
        if ((MSM_BOOT_MODE__FACTORY == get_boot_mode()) && (is_project(OPPO_14029) || is_project(OPPO_13095))) {
            set_backlight_pwm(0);
        }
/* OPPO 2014-05-30 yxq add end */
	}
	return;
}

module_i2c_driver(lm3630_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630");
MODULE_AUTHOR("G.Shark Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
