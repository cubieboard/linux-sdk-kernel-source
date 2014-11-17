######################################
#
#    RoboPeak USB LCD Display Linux Driver
#    
#    Copyright (C) 2009 - 2013 RoboPeak Team
#    This file is licensed under the GPL. See LICENSE in the package.
#
#    http://www.robopeak.net
#
#    Author Shikai Chen
#
######################################


DRIVER_NAME := rp_usbdisplay

ccflags-y = -Idrivers/video/rpusbdisp/drivers/common -Idrivers/video/rpusbdisp/drivers/linux-driver/src


obj-m := $(DRIVER_NAME).o

DRIVER_FILES := src/main.o \
                src/usbhandlers.o \
		src/fbhandlers.o \
		src/touchhandlers.o

$(DRIVER_NAME)-objs:= $(DRIVER_FILES)
