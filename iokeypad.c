#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/input.h>



struct iokeypad_dev {
    
    struct delayed_work work;
    int gpio_in_num;
    int gpio_out_num;
    unsigned int *inp_gpios;
    unsigned int *oup_gpios;
    u32 *keymap;
    int *matrix;
    int *old_matrix;
};
struct input_dev *input_iokeypad;

static void disable_input_irqs(struct iokeypad_dev *iokeypad)
{
    // pr_info("iokeypad IRQ disabled!\n");
    for (int i = 0; i < iokeypad->gpio_in_num; i++)
        disable_irq_nosync(gpio_to_irq(iokeypad->inp_gpios[i]));
}

static void enable_input_irqs(struct iokeypad_dev *iokeypad)
{
    // struct gpio_descs *gpios = iokeypad->in_gpios;
    // pr_info("iokeypad IRQ enabled!\n");
    for (int i = 0; i < iokeypad->gpio_in_num; i++)
        enable_irq(gpio_to_irq(iokeypad->inp_gpios[i]));
}

void irq_bottom_half(struct work_struct *work)
{
    struct iokeypad_dev *iokeypad = container_of(work, struct iokeypad_dev, work.work);
    // char scan_output[10];
    int change = -1;
    int col, row;
    // char operator;


    // pr_info("iokeypad Rising edge detected!\n");

    for (int i = 0; i < iokeypad->gpio_out_num; i++)
        gpio_set_value(iokeypad->oup_gpios[i], 1);
    // pr_info("iokeypad OUTPUT HIGH!\n");

    for(int i=0; i < iokeypad->gpio_out_num; i++){
        gpio_set_value(iokeypad->oup_gpios[i], 0);

        for(int j=0; j < iokeypad->gpio_in_num; j++)
            iokeypad->matrix[(i*iokeypad->gpio_in_num)+j] = gpio_get_value_cansleep(iokeypad->inp_gpios[j]);
        
        gpio_set_value(iokeypad->oup_gpios[i], 1);
    }

    // Scan difference keypad status 
    for (int i = 0; i < iokeypad->gpio_out_num; i++){
        for (int j = 0; j < iokeypad->gpio_in_num; j++){
            if (iokeypad->matrix[(i*iokeypad->gpio_in_num)+j] != iokeypad->old_matrix[(i*iokeypad->gpio_in_num)+j]){
                iokeypad->old_matrix[(i*iokeypad->gpio_in_num)+j] = iokeypad->matrix[(i*iokeypad->gpio_in_num)+j];
                if(iokeypad->matrix[(i*iokeypad->gpio_in_num)+j]==0){
                    change = (i*iokeypad->gpio_in_num)+j;
                    col = j;
                    row = i;
                }
                else if (iokeypad->matrix[(i*iokeypad->gpio_in_num)+j]==1){
                    change = 200+(i*iokeypad->gpio_in_num)+j;
                    col = j;
                    row = i;
                }
            }
        }
    }  

    // print difference keypad status
    if (change >= 0){
        if (change > 199){
            //pr_info("[%d],[%d] = 1\n", col, row);
            input_report_key(input_iokeypad, iokeypad->keymap[(change-200)], 0);
        }else if (change < 200){
            //pr_info("[%d],[%d] = 0\n", col, row);
            input_report_key(input_iokeypad, iokeypad->keymap[change], 1);
        }
        //memory += cauculator;
        change = -1;
        input_sync(input_iokeypad);
    }

    for (int i = 0; i < iokeypad->gpio_out_num; i++)
        gpio_set_value(iokeypad->oup_gpios[i], 0);

    enable_input_irqs(iokeypad);
}

static irqreturn_t irq_top_half(int irq, void *p)
{
    struct iokeypad_dev *iokeypad = p;
    // unsigned long flags;

    // spin_lock_irqsave(&iokeypad->lock, flags);

    disable_input_irqs(iokeypad);
    schedule_delayed_work(&(iokeypad -> work), 10);
    return IRQ_HANDLED;
}

static int iokeypad_init(struct platform_device *pdev,
				   struct iokeypad_dev *iokeypad)
{
    struct device *dev = &(pdev-> dev);
    int ret = 0,i, err;

    for (i = 0; i < iokeypad->gpio_in_num; i++) {
        // iokeypad->inp_gpios[i] = of_get_named_gpio(np, "iokeypad_in-gpios", i);
        pr_info("iokeypad->inp_gpios[%d] = %d", i,iokeypad->inp_gpios[i]);
		err = gpio_request(iokeypad->inp_gpios[i], "iokeypad_in");
		if (err) {
			dev_err(dev,
				"failed to request GPIO%d for COL%d\n",
				iokeypad->inp_gpios[i], i);
			return 0;
		}

		gpio_direction_input(iokeypad->inp_gpios[i]);
	}

    for(i=0; i < iokeypad->gpio_in_num; i++){
        ret = request_any_context_irq(gpio_to_irq(iokeypad->inp_gpios[i]), irq_top_half, 
                IRQF_TRIGGER_FALLING, "iokeypad_in", iokeypad);
        if (ret < 0) {
            dev_err(dev, "Failed to request IRQ.\n");
            return ret;
        }
    }
    // Get gpio 
    // out_gpios = devm_gpiod_get_array(dev, "iokeypad_out", GPIOD_OUT_LOW);
    // if (IS_ERR(out_gpios)) {
    //     dev_err(dev, "Failed to get gpio descriptor%d.\n",i);
    // return PTR_ERR(out_gpios);
    // }

    for (i = 0; i < iokeypad->gpio_out_num; i++) {
        // iokeypad->oup_gpios[i] = of_get_named_gpio(np, "iokeypad_out-gpios", i);
        pr_info("iokeypad->oup_gpios[%d] = %d", i,iokeypad->oup_gpios[i]);
		err = gpio_request(iokeypad->oup_gpios[i], "iokeypad_out");
		if (err) {
			dev_err(dev,
				"failed to request GPIO%d for OUTPUT%d\n",
				iokeypad->oup_gpios[i], i);
			return 0;
		}

		gpio_direction_output(iokeypad->oup_gpios[i], 0);
	}

    return 1;
}

// static int keymap_parse(int input, int output,struct input_dev *input_iokeypad, u32 *keymap)
// {
//     struct device *dev = input_iokeypad->dev.parent;
//     struct device_node *np = dev->of_node;
//     // int *keymap;
//     int retval, size;
//     size = of_property_count_u32_elems(np, "iokeypad,keymap");
//     pr_info("Num of size = %d\n", size);
//     pr_info("Num of %d*%d = %d\n", input, output, (input)*(output));
//     if(size != (input)*(output)) {
//         dev_err(dev, "number of keymap not correct!\n");
// 		return -EINVAL;
//     }
//     keymap = kmalloc_array(size, sizeof(u32), GFP_KERNEL);
//     if (!keymap){
//         pr_info("iokeypad keymap kmalloc fail");
//         return -ENOMEM;
//     }
		
//     retval = of_property_read_u32_array(np, "iokeypad,keymap", 
//                                                 keymap, size);
//     if (retval) {
// 		dev_err(dev, "failed to read %s property: %d\n",
// 			"iokeypad,keymap", retval);
// 		goto out;
// 	}

//     retval = 0;
//     return retval;
// out:
// 	kfree(keymap);
// 	return retval;
// }

static int iokeypad_probe(struct platform_device *pdev)
{
    struct device *dev = &(pdev-> dev);
    struct iokeypad_dev *iokeypad;
    struct device_node *np = dev->of_node;
    unsigned int *gpio_in;
    unsigned int *gpio_out;
    int *gpio_matrix;
    int *gpio_old_matrix;
    int err = 0,size;
    u32 *minimap;
    

    // Set up iokeypad memory
    iokeypad = kzalloc(sizeof(struct iokeypad_dev), GFP_KERNEL);
    if (!iokeypad) {
        dev_err(dev, "Failed to allocate memory.\n");
	    return -ENOMEM;
    }
    
    // count number of gpios input ro output 
    iokeypad->gpio_in_num = of_gpio_named_count(np, "iokeypad_in-gpios");
    pr_info("Num of %s gpio = %d\n", "iokeypad_in", iokeypad->gpio_in_num);
    iokeypad->gpio_out_num = of_gpio_named_count(np, "iokeypad_out-gpios");
    pr_info("Num of %s gpio = %d\n", "iokeypad_out", iokeypad->gpio_out_num);


    // Set up gpio arrays memory with kcalloc
    gpio_in = kcalloc(iokeypad->gpio_in_num, sizeof(unsigned int), GFP_KERNEL);
    if (!gpio_in) {
		dev_err(dev, "could not allocate memory for gpios\n");
	}
    // pr_info("iokeypad kzalloc gpioin passed");

    for (int i = 0; i < iokeypad->gpio_in_num; i++) {
        // pr_info("iokeypad->inp_gpios[%d] = %d", i,gpio_in[i]);
        gpio_in[i] = of_get_named_gpio(np, "iokeypad_in-gpios", i);
    }
    
	gpio_out = kcalloc(iokeypad->gpio_out_num, sizeof(unsigned int), GFP_KERNEL);

    for (int i = 0; i < iokeypad->gpio_out_num; i++){
        gpio_out[i] = of_get_named_gpio(np, "iokeypad_out-gpios", i);
    }
    gpio_matrix = kcalloc(iokeypad->gpio_out_num + iokeypad->gpio_in_num,
                            sizeof(int) , GFP_KERNEL);
    gpio_old_matrix = kcalloc(iokeypad->gpio_out_num + iokeypad->gpio_in_num,
                            sizeof(int) , GFP_KERNEL);

    for (int i = 0; i < iokeypad->gpio_out_num; i++){
        for (int j = 0; j < iokeypad->gpio_in_num; j++){
            gpio_matrix[(i*iokeypad->gpio_in_num)+j] = 1;
            gpio_old_matrix[(i*iokeypad->gpio_in_num)+j] = 1;
        }
    }
    input_iokeypad = input_allocate_device();

    // pr_info("iokeypad kzalloc success");

    iokeypad->inp_gpios = gpio_in;
    iokeypad->oup_gpios = gpio_out;
    iokeypad->matrix = gpio_matrix;
    iokeypad->old_matrix = gpio_old_matrix;

    input_iokeypad->name		= pdev->name;
    input_iokeypad->dev.parent	= &pdev->dev;

    size = of_property_count_u32_elems(np, "iokeypad,keymap");
    // pr_info("Num of size = %d\n", size);

    if(size != (iokeypad->gpio_in_num)*(iokeypad->gpio_out_num)) {
        dev_err(dev, "number of minimap not correct!, size = %d, input*output = %d\n",
                size, (iokeypad->gpio_in_num)*(iokeypad->gpio_out_num));
		return -EINVAL;
    }
    if(size > 200)
        dev_err(dev, "keypad size is too big!, size = %d\n",size);
    minimap = kmalloc_array(size, sizeof(u32), GFP_KERNEL);
    if (!minimap){
        pr_info("iokeypad minimap kmalloc fail");
        return -ENOMEM;
    }
		
    err = of_property_read_u32_array(np, "iokeypad,keymap", 
                                                minimap, size);
    if (err) {
		dev_err(dev, "failed to read %s property: %d\n",
			"iokeypad,keymap", err);
	}

    iokeypad->keymap = minimap;
    // pr_info("iokeypad keypad parse success");
    __set_bit(EV_KEY, input_iokeypad->evbit);
    // pr_info("iokeypad keypad ev success");
    // __set_bit(EV_REP, input_iokeypad->evbit);
    // input_set_capability(input_iokeypad, EV_MSC, MSC_SCAN);
    
    for (int i=0; i < (iokeypad->gpio_in_num)*(iokeypad->gpio_out_num); i++){
        pr_info("iokeypad keymap %d = %d",i ,iokeypad->keymap[i]);
        __set_bit(iokeypad->keymap[i], input_iokeypad->keybit);
    }
    input_set_drvdata(input_iokeypad, iokeypad);
    pr_info("iokeypad keypad set key success");
    err = input_register_device(input_iokeypad);
	if (err)
		goto err_free_gpio;

    // request correct gpio number and register irq number
    err = iokeypad_init(pdev, iokeypad);
    if(err!=1){
        pr_info("Init iokeypad INPUT Fail\n");
        return 0;
    }

    // spin_lock_init(&iokeypad->lock);
    INIT_DELAYED_WORK(&iokeypad->work, irq_bottom_half);
    
    platform_set_drvdata(pdev, iokeypad);
	printk("Probe iokeypad success .\n");
    return 0;

err_free_gpio:
	input_free_device(input_iokeypad);
	kfree(iokeypad);
	return err;
}

static const struct of_device_id iokeypad_ids[] = {
    {.compatible = "iokeypad",},
    {}
};

static struct platform_driver iokeypad_driver = {
    .driver = {
        .name = "iokeypad",
	    .of_match_table = iokeypad_ids,
    },
    .probe = iokeypad_probe
};

module_platform_driver(iokeypad_driver);

MODULE_LICENSE("GPL");