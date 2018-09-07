DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../../../

PROJECT_NAME:=oc
PROJECT_SOURCEFILES+=../command.c

HAVE_BANKING=1

all: $(PROJECT_NAME)

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include

flash: all
	cc-tool -ew $(PROJECT_NAME).hex