#****************************************************************************
# Copyright (C) 2001-2010  PEAK System-Technik GmbH
#
# linux@peak-system.com
# www.peak-system.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# Maintainer(s): Stephane Grosjean <s.grosjean@peak-system.com>
# Contributions: Klaus Hitschler <klaus.hitschler@gmx.de>
#                Pablo Yanez Trujillo <yanez@pse.de> cross-compile
#****************************************************************************

#
# Makefile - global Makefile for all components
#
# $Id$
#

# CROSS-COMPILING SETTINGS
#
# You need a cross-compiler. You can build one with crosstool
# http://www.kegel.com/crosstool/current/doc/crosstool-howto.html
#
# These variables work with the toolchains created by crosstool

# defines the architecture. For example 'arm' for an ARM system
#export ARCH=arm

# the path and prefix of the cross-compiler
#export CROSS_COMPILE=/home/yanez/matrix500/arm-9tdmi-linux-gnu/bin/arm-9tdmi-linux-gnu-
export CC=$(CROSS_COMPILE)gcc

# MACROS AND DEFINES
PCAN_BASIC = libpcanbasic

ifneq ($(PCAN_BASIC),)
#PWD = $(shell pwd)
# @make -C $(PCAN_BASIC) PCAN_ROOT=$(PWD) $1
define make-pcanbasic
@$(MAKE) -C $(PCAN_BASIC) $1
endef
else
define make-pcanbasic
endef
endif

define do-make
@$(MAKE) -C driver $1
@$(MAKE) -C lib $1
@$(MAKE) -C test $1
$(call make-pcanbasic, $1)
endef

define make-all
$(call do-make, all)
endef

define make-clean
$(call do-make, clean)
endef

define make-install
$(call do-make, install)
endef

define make-xeno
$(call do-make, xeno)
endef

define make-rtai
$(call do-make, rtai)
endef

define make-uninstall
$(call do-make, uninstall)
endef

all:
	$(make-all)

chardev: all

# do-build lib, test (and pcanbasic) even if useless in netdev mode:
# those binaries have to exist for the install procedure to complete.
netdev:
	@$(MAKE) -C driver netdev
	@$(MAKE) -C lib
	@$(MAKE) -C test
ifneq ($(PCAN_BASIC),)
	@$(MAKE) -C $(PCAN_BASIC)
endif

clean:
	$(make-clean)

install:
	$(make-install)
 
uninstall:
	$(make-uninstall)

xeno:
	$(make-xeno)

rtai:
	$(make-rtai)

pcanbasic:
ifneq ($(PCAN_BASIC),)
	$(call make-pcanbasic, all)
else
	@echo Warning: PCAN_BASIC not defined in that version.
endif
# end

# DO NOT DELETE
