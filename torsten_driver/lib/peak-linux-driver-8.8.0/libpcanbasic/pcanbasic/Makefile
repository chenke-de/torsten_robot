#  Makefile
#
#  ~~~~~~~~~~~~
#
#  PCANBasic
#
#  ~~~~~~~~~~~~
#
#  ------------------------------------------------------------------
#  Last change: $Date: 2019-04-04 14:47:45 +0200 (jeu., 04 avril 2019) $
#  Language: make
#  ------------------------------------------------------------------
#
#  Copyright (C) 1999-2016  PEAK-System Technik GmbH, Darmstadt
#  more Info at http://www.peak-system.com
#  ------------------------------------------------------------------
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
#****************************************************************************

# This file is part of the standalone package of PCAN-Basic for Linux
# It is not past of the peak-linux-driver v8.x package.
# If present, it should define:
#
# CONFIG_PCAN_VERSION=xxx
-include src/pcan/.config

# if CONFIG_PCAN_VERSION is not defined, then we run from the pcan driver
# package
ifeq ($(CONFIG_PCAN_VERSION),)

PCAN_ROOT := $(shell cd ../..; pwd)

# pcanbasic is built from the pcan driver package environment:
# always build the latest version of pcanbasic.
include Makefile_latest.mk

else

# otherwise, consider version of the installed pcan driver:

# if /usr/include/pcanfd.h does exist, then build latest version of pcanbasic.
PCANFD_HEADER ?= /usr/include/pcanfd.h

ifneq ($(wildcard $(PCANFD_HEADER)),)

$(info Considering PCANBasic library with FD support (PCAN driver >= 8.0))
include Makefile_latest.mk

else

# otherwise
# if /usr/include/pcan.h does exist, then build legacy version of pcanbasic.
PCAN_HEADER ?= /usr/include/pcan.h

ifneq ($(wildcard $(PCAN_HEADER)),)

$(warning Considering PCANBasic library WITHOUT FD support (PCAN driver < 8.0))
include Makefile_legacy.mk

else

#$(error Unable to build pcanbasic for Linux: install the pcan driver first)
all install uninstall:
	@echo "Unable to build pcanbasic for Linux: install the pcan driver first"
	@exit 1

# clean any tmp files, even if pcan is not installed
clean:
	-for f in `find . -name "*.o" -o -name "*.a" -o -name "*.so" -o -name "*.so.*"`; do rm -f $$f; done
endif

endif
endif
