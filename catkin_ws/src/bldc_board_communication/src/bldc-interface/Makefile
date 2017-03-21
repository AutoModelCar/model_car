
TARGET = bldc-interface

CROSS_COMPILE_PREFIX = 
CC      = $(CROSS_COMPILE_PREFIX)gcc
CPP     = $(CROSS_COMPILE_PREFIX)g++
LD      = $(CROSS_COMPILE_PREFIX)g++
SIZE    = $(CROSS_COMPILE_PREFIX)size
GDB     = $(CROSS_COMPILE_PREFIX)gdb
OBJ_CPY = $(CROSS_COMPILE_PREFIX)objcopy

LIBS = m pthread usb-1.0
LIB_PATHS = 
INCLUDES = src/ \

# Flags
CPPFLAGS = -c -O0 -ggdb -Wall -Wextra -fmessage-length=0 -std=c++11
LINKERFLAGS = 


CPP_SUFFIX = .cpp
OBJ_SUFFIX = .o
DEP_SUFFIX = .d
OBJ_DIR = obj/

IGNORE_STRINGS += "*test* *archive*"
CPP_FILES      += $(sort $(filter-out $(IGNORE_STRINGS), $(shell find src -name "*$(CPP_SUFFIX)")))
CPP_OBJ_FILES  += $(addsuffix $(OBJ_SUFFIX), $(addprefix $(OBJ_DIR), $(CPP_FILES)))
DEP_FILES      += $(addprefix $(OBJ_DIR), $(addsuffix $(DEP_SUFFIX), $(CPP_FILES)))

INCLUDE_CMD = $(addprefix -I, $(INCLUDES))
LIB_CMD = $(addprefix -l, $(LIBS))
LIB_PATH_CMD = $(addprefix -L, $(LIB_PATHS))

.phony: all clean flash

all: $(TARGET)

clean:
	$(SILENT) rm -rf $(OBJ_DIR) $(TARGET)

dbg:
	@echo $(CPP_OBJ_FILES)


$(TARGET): $(CPP_OBJ_FILES)
	$(SILENT) echo linking $(target)
	$(SILENT) $(LD) -o $@ $^ $(LINKERFLAGS) $(LIB_PATH_CMD) $(LIB_CMD)
	$(SILENT) $(SIZE) $@
	@ echo done

$(OBJ_DIR)%$(CPP_SUFFIX)$(OBJ_SUFFIX): %$(CPP_SUFFIX)
	@echo building $<
	@ mkdir -p $(dir $@)
	@ $(CPP) $(CPPFLAGS) $(INCLUDE_CMD) -MM -MF $(OBJ_DIR)$<.d -c $<
	@ mv -f $(OBJ_DIR)$<.d $(OBJ_DIR)$<.d.tmp
	@ sed -e 's|.*:|$@:|' < $(OBJ_DIR)$<.d.tmp > $(OBJ_DIR)$<.d
	@ rm -f $(OBJ_DIR)$<.d.tmp
	
	$(SILENT) $(CPP) $(CPPFLAGS) $(INCLUDE_CMD) -o $@ -c $<
	
-include $(DEP_FILES)
