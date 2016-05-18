CONTIKI = ../..

all: hallway-test

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include
