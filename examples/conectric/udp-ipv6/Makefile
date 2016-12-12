DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

HAVE_BANKING=1

PROJECT_SOURCEFILES += ping6.c rimestats.c

CONTIKI_PROJECT = client server

all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..
CONTIKI_WITH_IPV6 = 1
RPL_CONF_WITH_NON_STORING = 1
include $(CONTIKI)/Makefile.include
