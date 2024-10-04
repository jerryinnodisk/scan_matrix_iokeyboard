# scan_matrix_iokeyboard

## 1.Add iokeypad driver under linux-imx/git/drivers/input/keyboard

## 2.Add discribe under Kconfig
```
config IOKEYPAD
	tristate "IOKEYPAD"
	depends on GPIOLIB || COMPILE_TEST
```
## 3.Add make command under Makefile 
```
obj-$(CONFIG_IOKEYPAD)			+= iokeypad.o
```

## 4.Set y under kernel config
```
CONFIG_IOKEYPAD=y
```

## 5.Compile linux-imx 
```
$ bitbake linux-imx -c compile -f kernel_configme
$ bitbake linux-imx 
```