DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../../../

HAVE_BANKING=1

all: sw

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include

flash: all
	cc-tool -ew sw.hex