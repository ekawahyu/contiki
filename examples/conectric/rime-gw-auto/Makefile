DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../../../

HAVE_BANKING=1

all: gw-auto

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include

flash: all
	cc-tool -ew gw-auto.hex
