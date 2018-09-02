
default: all


TARGET_EXEC ?= lcd

BUILD_DIR ?= bin

SRCS := $(shell find . -name '*.cpp' -or -name '*.c' -or -name '*.s')
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find . -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

#LIBS= -lpthread -lm
#LIBS= -lpigpio -lpthread -lm
#LIBS= -lwiringPi -lm -lpthread
#LIBS= -lbcm2835 -lm
LIBS= -lwiringPi -lwiringPiDev -lpthread -lrt -lm -lcrypt
CCFLAGS ?= $(INC_FLAGS) $(LIBS) -MMD -MP -g -Os -w -ffreestanding
LDFLAGS += -lstdc++


all: $(BUILD_DIR)/$(TARGET_EXEC)

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) -std=gnu11 -lstdc $(CCFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) -std=c++11 -lstdc++ $(CCFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
