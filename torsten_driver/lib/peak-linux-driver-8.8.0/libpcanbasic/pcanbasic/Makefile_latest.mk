#  Makefile
#
#  ~~~~~~~~~~~~
#
#  PCANBasic
#
#  ~~~~~~~~~~~~
#
#  ------------------------------------------------------------------
#  Last changed by:	$Author: Stephane $
#  Last change: $Date: 2019-04-09 15:00:04 +0200 (mar., 09 avril 2019) $
#  Language: make
#  ------------------------------------------------------------------
#
#  Copyright (C) 1999-2018  PEAK-System Technik GmbH, Darmstadt
#  more Info at http://www.peak-system.com
# ------------------------------------------------------------------
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
#****************************************************************************

# Commands
CC = gcc
LN = ln -sf

# libpcanbasic C default flags
CFLAGS = -fPIC -shared -O2
CFLAGS += -Wall -Wcast-align -Wcast-qual -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wunused

# use -Wshadow with gcc > 4.6 only
#CFLAGS += -Wshadow

# Paths and files
SRC = src

# pcan driver/lib uapi root dir
PCAN_ROOT ?= $(SRC)/pcan

# libpcanbasic compiles libpcanfd source files
LIBPCANFD_SRC = $(PCAN_ROOT)/lib/src/libpcanfd.c
LIBPCANFD_INC = -I$(PCAN_ROOT)/driver -I$(PCAN_ROOT)/lib
LIBPCANFD_OBJ = libpcanfd.o

# libpcanfd compile option 
RT ?= NO_RT

# libpcanbasis cource files
FILES   = $(SRC)/libpcanbasic.c
FILES   += $(SRC)/pcaninfo.c
FILES   += $(SRC)/pcanlog.c
FILES   += $(SRC)/pcbcore.c
FILES   += $(SRC)/pcblog.c
FILES   += $(SRC)/pcbtrace.c
ALL_OBJ :=  $(foreach f,$(FILES),$(SRC)/$(basename $(notdir $(f))).o)

# get build version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
VERSION_FILE = 'src/version.h'
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION))
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION))
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION))

# targets
NAME = libpcanbasic
EXT = .so
TARGET_SHORT = $(NAME)$(EXT)
TARGET = $(TARGET_SHORT).$(MAJOR).$(MINOR).$(PATCH)
SONAME = $(TARGET_SHORT).$(MAJOR)
SONAME_OLD = $(TARGET_SHORT).0

# Define flags for XENOMAI installation only
ifeq ($(RT), XENOMAI)
RT_DIR ?= /usr/xenomai
RT_CONFIG ?= $(RT_DIR)/bin/xeno-config

SKIN := rtdm
RT_CFLAGS := $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
RT_LDFLAGS := -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --skin $(SKIN) --ldflags)
endif

# Define flags for RTAI installation only
ifeq ($(RT), RTAI)
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config

SKIN := lxrt
RT_CFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

# Complete flags
CFLAGS += -D$(RT) $(LIBPCANFD_INC) $(RT_CFLAGS)
LDFLAGS += -lm $(RT_LDFLAGS)

# Installation directory
LIBPATH = $(DESTDIR)/usr/lib

#********** entries *********************

all: message $(TARGET_SHORT)

$(TARGET_SHORT): $(TARGET)
	$(LN) $(TARGET) $(TARGET_SHORT)

$(TARGET): $(ALL_OBJ) $(SRC)/$(LIBPCANFD_OBJ)
	$(CC) -shared -Wl,-soname,$(TARGET_SHORT) -o $@ $^ $(LDFLAGS)

$(SRC)/$(LIBPCANFD_OBJ): $(LIBPCANFD_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

$(SRC)/%.o: $(SRC)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.* *.so $(LIBPCANFD_OBJ)

.PHONY: message
message:
	@echo "***"
	@echo "*** Making PCANBasic library with FD support (PCAN driver >= 8.0)"
	@echo "***"
	@echo "*** target=$(NAME)" 
	@echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@echo "*** PCAN_ROOT=$(PCAN_ROOT)"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"

xeno:
	$(MAKE) RT=XENOMAI

rtai:
	$(MAKE) RT=RTAI

#********** these entries are reserved for root access only *******************
install: $(TARGET)
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME_OLD)
	$(LN) $(LIBPATH)/$(SONAME) $(LIBPATH)/$(TARGET_SHORT)
	cp PCANBasic.h $(DESTDIR)/usr/include/PCANBasic.h
	chmod 644 $(DESTDIR)/usr/include/PCANBasic.h
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif
  
uninstall:
	-rm $(DESTDIR)/usr/include/PCANBasic.h
	-rm $(LIBPATH)/$(TARGET_SHORT)
	-rm $(LIBPATH)/$(SONAME_OLD)
	-rm $(LIBPATH)/$(SONAME)
	-rm $(LIBPATH)/$(TARGET)
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif

uninstall-purge: uninstall
	-rm $(LIBPATH)/$(TARGET_SHORT).*
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif
