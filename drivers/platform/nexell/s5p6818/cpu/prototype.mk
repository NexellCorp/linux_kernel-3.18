#
# (C) Copyright 2009
# jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

obj-y += prototype/base/nx_bit_accessor.o

obj-y += \
		prototype/module/nx_clkpwr.o		\
   		prototype/module/nx_rstcon.o		\
   		prototype/module/nx_tieoff.o		\
		prototype/module/nx_mcus.o		\
		prototype/module/nx_timer.o		\
		prototype/module/nx_gpio.o		\
		prototype/module/nx_alive.o		\
		prototype/module/nx_clkgen.o		\
		prototype/module/nx_ecid.o		\
		prototype/module/nx_rtc.o

obj-$(CONFIG_NXP_DISPLAY)				+= 	prototype/module/nx_displaytop.o		\
								   			prototype/module/nx_disptop_clkgen.o	\
											prototype/module/nx_dualdisplay.o	\
								   			prototype/module/nx_mlc.o			\
								   			prototype/module/nx_dpc.o

obj-$(CONFIG_NXP_DISPLAY_LVDS)			+= 	prototype/module/nx_lvds.o
obj-$(CONFIG_NXP_DISPLAY_RESCONV)		+= 	prototype/module/nx_resconv.o
obj-$(CONFIG_NXP_DISPLAY_HDMI)			+= 	prototype/module/nx_hdmi.o
obj-$(CONFIG_NXP_DISPLAY_HDMI)			+= 	prototype/module/nx_ecid.o
obj-$(CONFIG_NXP_DISPLAY_MIPI)			+=  prototype/module/nx_mipi.o

obj-$(CONFIG_RTC_DRV_NXP)				+= 	prototype/module/nx_rtc.o

obj-$(CONFIG_HAVE_PWM)					+= 	prototype/module/nx_pwm.o
obj-$(CONFIG_NXP_ADC)					+= 	prototype/module/nx_adc.o
obj-$(CONFIG_VIDEO_NXP_CAPTURE)			+=  prototype/module/nx_displaytop.o \
																				prototype/module/nx_dualdisplay.o	\
																				prototype/module/nx_disptop_clkgen.o	\
																				prototype/module/nx_vip.o	\
																				prototype/module/nx_hdmi.o	\
								   											prototype/module/nx_mlc.o	\
								   											prototype/module/nx_dpc.o

obj-$(CONFIG_SLSIAP_BACKWARD_CAMERA)	+=  prototype/module/nx_vip.o
obj-$(CONFIG_NXP_OUT_HDMI)				+= 	prototype/module/nx_hdmi.o
obj-$(CONFIG_NXP_OUT_HDMI)				+= 	prototype/module/nx_ecid.o

obj-$(CONFIG_NXP_M2M_SCALER)			+=  prototype/module/nx_scaler.o
obj-$(CONFIG_NXP_CAPTURE_MIPI_CSI)		+=  prototype/module/nx_mipi.o
obj-$(CONFIG_NXP_MP2TS_IF)				+=  prototype/module/nx_mpegtsi.o

obj-$(CONFIG_RTC_DRV_NXP)				+=  prototype/module/nx_rtc.o

obj-$(CONFIG_PPM_NXP) 					+=  prototype/module/nx_ppm.o
obj-$(CONFIG_SND_NXP_PDM)               +=  prototype/module/nx_pdm.o
obj-$(CONFIG_SENSORS_NXP_TMU) 			+=  prototype/module/nx_tmu.o
obj-$(CONFIG_NXP_DISPLAY_TVOUT)			+= 	prototype/module/nx_hdmi.o
obj-$(CONFIG_NXP_M2M_DEINTERLACER) +=  prototype/module/nx_deinterlace.o
