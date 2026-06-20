
mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
Pid.o

APPL_LIBS += -lm

APPL_DIRS += $(makefile_path)Pid

INCLUDES += \
    -I$(makefile_path)Pid \
