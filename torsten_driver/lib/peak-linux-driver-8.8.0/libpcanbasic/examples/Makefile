#

PWD := $(shell pwd)

export PCANBASIC_ROOT := $(shell cd $(PWD)/../pcanbasic; pwd)

all clean install uninstall xeno rtai:
# if any Makefile failed, stop building all...
	@for m in `find . -mindepth 2 -name Makefile`; do\
		$(MAKE) -C `dirname $$m` $@; \
		if [ $$? -ne 0 ]; then exit 1; fi; \
	done
