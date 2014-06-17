CONTIKI=../../../../../contiki
TARGETDIRS=../../../platform
TARGET=tyndall

all:	clock-tests

PROJECT_SOURCEFILES += modbusProtocol.c modbus-api.c rs485-dev.c

ifdef MAC
CFLAGS += -DMAC_ADDR=\"$(MAC)\"
endif

include $(CONTIKI)/Makefile.include
