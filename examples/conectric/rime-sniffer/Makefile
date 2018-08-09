DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../../../

CONTIKI_PROJECT = sniffer
PROJECT_SOURCEFILES += stub-rdc.c

HAVE_BANKING=1

all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include

flash: all
	cc-tool -ew $(CONTIKI_PROJECT).hex