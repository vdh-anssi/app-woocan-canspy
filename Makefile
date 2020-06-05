###################################################################
# about the application name and path
###################################################################

APP_NAME ?=canspy
DIR_NAME = canspy

PROJ_FILES = ../../
BIN_NAME = $(APP_NAME).bin
HEX_NAME = $(APP_NAME).hex
ELF_NAME = $(APP_NAME).elf

# SDK helper Makefiles inclusion
-include $(PROJ_FILES)/m_config.mk
-include $(PROJ_FILES)/m_generic.mk

# use an app-specific build dir
APP_BUILD_DIR = $(BUILD_DIR)/apps/$(DIR_NAME)

###################################################################
# About the compilation flags
###################################################################

CFLAGS := $(APPS_CFLAGS)
CFLAGS += -Isrc/ -MMD -MP

###################################################################
# About the link step
###################################################################

LDFLAGS += $(EXTRA_LDFLAGS) -L$(APP_BUILD_DIR)
LD_LIBS += -lstd -lcan

###################################################################
# okay let's list our source files and generated files now
###################################################################


CSRC_DIR = src
SRC = $(wildcard $(CSRC_DIR)/*.c)
OBJ = $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(SRC))
DEP = $(OBJ:.o=.d)

OUT_DIRS = $(dir $(OBJ))

LDSCRIPT_NAME = $(APP_BUILD_DIR)/$(APP_NAME).ld

# file to (dist)clean
# objects and compilation related
TODEL_CLEAN += $(OBJ) $(DEP) $(LDSCRIPT_NAME)
# targets
TODEL_DISTCLEAN += $(APP_BUILD_DIR)

.PHONY: app

############################################################
# explicit dependency on the application libs and drivers
# compiling the application requires the compilation of its
# dependencies
###########################################################

## library dependencies
LIBDEP := $(BUILD_DIR)/libs/libstd/libstd.a

libdep: $(LIBDEP)

$(LIBDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)libs/$(patsubst lib%.a,%,$(notdir $@))


# drivers dependencies
SOCDRVDEP := $(BUILD_DIR)/drivers/libcan/libcan.a

socdrvdep: $(SOCDRVDEP)

$(SOCDRVDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)drivers/socs/$(SOC)/$(patsubst lib%.a,%,$(notdir $@))

# board drivers dependencies
BRDDRVDEP    :=

brddrvdep: $(BRDDRVDEP)

$(BRDDRVDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)drivers/boards/$(BOARD)/$(patsubst lib%.a,%,$(notdir $@))

# external dependencies
EXTDEP    :=

extdep: $(EXTDEP)

$(EXTDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)externals


alldeps: libdep socdrvdep brddrvdep extdep

##########################################################
# generic targets of all applications makefiles
##########################################################

show:
	@echo
	@echo "\t\tAPP_BUILD_DIR\t=> " $(APP_BUILD_DIR)
	@echo
	@echo "C sources files:"
	@echo "\t\tSRC\t=> " $(SRC)
	@echo "\t\tOBJ\t=> " $(OBJ)
	@echo "\t\tDEP\t=> " $(DEP)
	@echo
	@echo "\t\tCFG\t=> " $(CFLAGS)


# all (default) build the app
all: $(APP_BUILD_DIR) alldeps app


# app build the hex and elf binaries
app: $(APP_BUILD_DIR)/$(ELF_NAME) $(APP_BUILD_DIR)/$(HEX_NAME)

# objet files and dependencies
$(APP_BUILD_DIR)/%.o: %.c
	$(call if_changed,cc_o_c)

# ELF file dependencies. libs are build separately and before.
# Be sure to add the libs to your config file!
$(APP_BUILD_DIR)/$(ELF_NAME): $(OBJ)
	$(call if_changed,link_o_target)

# same for hex
$(APP_BUILD_DIR)/$(HEX_NAME): $(APP_BUILD_DIR)/$(ELF_NAME)
	$(call if_changed,objcopy_ihex)

# same for bin. bin is not build but you can add it if you whish
$(APP_BUILD_DIR)/$(BIN_NAME): $(APP_BUILD_DIR)/$(ELF_NAME)
	$(call if_changed,objcopy_bin)

# special target to create the application build directory
$(APP_BUILD_DIR):
	$(call cmd,mkdir)


-include $(DEP)
