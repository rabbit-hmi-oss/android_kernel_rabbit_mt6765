#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/sched.h>

static DECLARE_WAIT_QUEUE_HEAD(m35774_run_thread_waiter);

enum ms35774_pinctrls {
  MS35774_PINCTRL_START,
  VM_H = MS35774_PINCTRL_START,
  VM_L,
  ENN_H,
  ENN_L,
  DIR_H,
  DIR_L,
  STEP_H,
  STEP_L,
  PDN_H,
  PDN_L,
  MS1_H,
  MS1_L,
  MS2_H,
  MS2_L,
  MS35774_PINCTRL_MAX
};

struct ms35774_data {
  struct device *dev;
  struct pinctrl *pinctrl;
  struct pinctrl_state *pc_state[MS35774_PINCTRL_MAX];
  struct work_struct init_work;

  int orientation;
  int orientation_request;
  bool run_flag;
};

struct ms35774_data *g_data;

static const char *pinctrl_state_name[MS35774_PINCTRL_MAX] = {
  "ms35774_vm_high",
  "ms35774_vm_low",
  "ms35774_enn_high",
  "ms35774_enn_low",
  "ms35774_dir_high",
  "ms35774_dir_low",
  "ms35774_step_high",
  "ms35774_step_low",
  "ms35774_pdn_high",
  "ms35774_pdn_low",
  "ms35774_ms1_high",
  "ms35774_ms1_low",
  "ms35774_ms2_high",
  "ms35774_ms2_low",
};

static int ms35774_parse_dts(struct ms35774_data *data)
{
  int i;

  data->pinctrl = devm_pinctrl_get(data->dev);
  if (IS_ERR(data->pinctrl)) {
    dev_err(data->dev, "[%s] fail to get pinctrl\n", __func__);
    return -ENODEV;
  }

  for (i=MS35774_PINCTRL_START;i<MS35774_PINCTRL_MAX;i++) {
    data->pc_state[i] = pinctrl_lookup_state(data->pinctrl, pinctrl_state_name[i]);
    if (IS_ERR(data->pc_state[i])) {
      dev_err(data->dev, "[%s] fail to get pinctrl %s\n", __func__, pinctrl_state_name[i]);
      return -ENODEV;
    }
  }

  /* init pin state */
  pinctrl_select_state(data->pinctrl, data->pc_state[VM_L]);
  pinctrl_select_state(data->pinctrl, data->pc_state[ENN_H]);
  pinctrl_select_state(data->pinctrl, data->pc_state[DIR_L]);
  pinctrl_select_state(data->pinctrl, data->pc_state[STEP_L]);
  pinctrl_select_state(data->pinctrl, data->pc_state[PDN_L]);
  pinctrl_select_state(data->pinctrl, data->pc_state[MS1_H]);
  pinctrl_select_state(data->pinctrl, data->pc_state[MS2_L]);

  return 0;
}

int m35774_run_thread_handler(void *_data)
{
  struct ms35774_data *data = (struct ms35774_data *)_data;
  int step, orientation;
  bool right = true;

  do {
    wait_event_interruptible(m35774_run_thread_waiter, data->run_flag);
    dev_info(data->dev, "[%s] run\n", __func__);
    if (data->orientation == data->orientation_request) {
      data->run_flag = false;
      dev_info(data->dev, "[%s] same orientation\n", __func__);
      continue;
    } else if (data->orientation > data->orientation_request) {
      pinctrl_select_state(data->pinctrl, data->pc_state[DIR_L]);
      step = 6 * (data->orientation - data->orientation_request);
    } else {
      pinctrl_select_state(data->pinctrl, data->pc_state[DIR_H]);
      step = 6 * (data->orientation_request - data->orientation);
    }
    dev_info(data->dev, "[%s] o:%d, or:%d, step:%d\n",
        __func__, data->orientation, data->orientation_request, step);
    pinctrl_select_state(data->pinctrl, data->pc_state[VM_H]);
    udelay(10*1000);
    pinctrl_select_state(data->pinctrl, data->pc_state[ENN_L]);
    while (step--) {
      pinctrl_select_state(data->pinctrl, data->pc_state[STEP_H]);
      udelay(50);
      pinctrl_select_state(data->pinctrl, data->pc_state[STEP_L]);
      udelay(360);
    }
    udelay(50);
    pinctrl_select_state(data->pinctrl, data->pc_state[ENN_H]);
    pinctrl_select_state(data->pinctrl, data->pc_state[VM_L]);
    data->run_flag = false;
    data->orientation = data->orientation_request;
    dev_info(data->dev, "[%s] done\n", __func__);
  } while (!kthread_should_stop());

  return 0;
}

static ssize_t show_m35774_orientation(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  struct ms35774_data *data = dev_get_drvdata(dev);

  return sprintf(buf, "%d\n", data->orientation);
}

static ssize_t store_m35774_orientation(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
  struct ms35774_data *data = dev_get_drvdata(dev);
  int orientation;

  if (kstrtoint(buf, 10, &orientation) == 0) {
    if (orientation >= 0 && orientation <= 180) {
      data->orientation_request = orientation;
      data->run_flag = true;
      wake_up_interruptible(&m35774_run_thread_waiter);
    } else {
      dev_err(data->dev,
          "[%s] wrong request orientation(%d), must be 0~180\n", __func__);
    }
  } else {
    dev_err(data->dev, "[%s] format error, request int 0~180\n", __func__);
  }

  return size;
}
static DEVICE_ATTR(orientation, 0664, show_m35774_orientation, store_m35774_orientation);

static void ms35774_init_work_handler(struct work_struct *work)
{
  struct ms35774_data *data = container_of(work, struct ms35774_data, init_work);

  data->orientation = 180;
  data->orientation_request = 0;
  data->run_flag = true;
  wake_up_interruptible(&m35774_run_thread_waiter);
  mdelay(2000);
  data->orientation_request = 90;
  data->run_flag = true;
  wake_up_interruptible(&m35774_run_thread_waiter);
}

static int ms35774_probe(struct platform_device *pdev)
{
  struct ms35774_platform_data *pdata = dev_get_platdata(&pdev->dev);
  struct ms35774_data *data;
  static struct task_struct *m35774_run_thread;
  int ret = 0;

  data = devm_kzalloc(&pdev->dev, sizeof(struct ms35774_data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;

  data->dev = &pdev->dev;
  platform_set_drvdata(pdev, data);

  dev_info(data->dev, "[%s] enter\n", __func__);
  ret = ms35774_parse_dts(data);
  if (ret) {
    dev_err(data->dev,"[%s] fail to parse dts\n", __func__);
    return ret;
  }

  data->run_flag = false;
  data->orientation = 0;
  data->orientation_request = 0;
  m35774_run_thread = kthread_run(m35774_run_thread_handler, data,
      "m35774_step_motor_run_thread");
  if (IS_ERR(m35774_run_thread)) {
    dev_err(data->dev, "[%s] fail to create motor run thread\n", __func__);
    return PTR_ERR(m35774_run_thread);
  }

  INIT_WORK(&data->init_work, ms35774_init_work_handler);
  ret = device_create_file(data->dev, &dev_attr_orientation);
  if (ret) {
    dev_err(data->dev, "[%s] fail to create sysfs attr! ret:%d\n", __func__, ret);
    return ret;
  }

  g_data = data;
  schedule_work(&data->init_work);
  dev_info(data->dev, "[%s] success!\n", __func__);

  return 0;
}

static void ms35774_shutdown(struct platform_device *pdev)
{
  struct ms35774_data *data = platform_get_drvdata(pdev);
  data->orientation_request = 90;
  data->run_flag = true;
  wake_up_interruptible(&m35774_run_thread_waiter);
  mdelay(1500);
}

static const struct of_device_id of_ms35774s_match[] = {
  { .compatible = "relmon,ms35774", },
  {},
};

MODULE_DEVICE_TABLE(of, of_ms35774s_match);

static struct platform_driver ms35774_driver = {
  .probe  = ms35774_probe,
  .shutdown = ms35774_shutdown,
  .driver  = {
    .name = "step_motor_ms35774",
    .of_match_table = of_ms35774s_match,
  },
};

module_platform_driver(ms35774_driver);

MODULE_DESCRIPTION("step motor ms35774 driver");
MODULE_LICENSE("GPL");
