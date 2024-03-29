#
# Makefile
#
##################################################################################
OS:=$(shell uname -s)
UNAME := $(shell uname)

##################################################################################
# library file name
APP_NAME = spline_interpolator
TEST_APP_NAME = unit_test

##################################################################################
# top directory name
TOP_DIR_NAME = $(APP_NAME)

##################################################################################
# compiler
# OS dependency
ifeq ($(OS),Linux)
	CXX:=g++ -std=c++03
else ifeq ($(OS),QNX)
  CXX:=QCC -Vgcc_ntox86_cpp
else
  $(error unknown OS $(OS))
endif

##################################################################################
# local directory
LIB_DIR = ./lib
BIN_DIR = ./bin
SRC_DIR = ./src
SRC_UTIL_DIR = $(SRC_DIR)/util

##################################################################################
# test directory
TEST_SRC_DIR = ./test
TEST_SRC_UTIL_DIR = $(TEST_SRC_DIR)/util

##################################################################################
# include directory
INCLUDE_DIR = ./include/$(TOP_DIR_NAME)
INCLUDE_USRLOCAL_DIR = /usr/local/include
INCLUDES = . $(INCLUDE_DIR)
INCLUDES += $(INCLUDE_USRLOCAL_DIR)
#
# OS dependency
ifeq ($(OS),QNX)
	INCLUDES += /usr/pkg/include
endif
#
# INCLUDES_PATH(add prefix -I)
INCLUDES_PATH = $(addprefix -I, $(INCLUDES))

##################################################################################
# library directoxry
LOCAL_LINK_DIRS = -L/usr/local/lib
LINK_DIRS = -L. -L/usr/lib
LINK_DIRS += $(LOCAL_LINK_DIRS)
#
# link (pay attension to linking-order)
LINK = -lm
#
# OS dependency
ifeq ($(OS),Linux)
  LINK+=-lpthread -ldl
endif
#
# for unit_test
LINK_GTEST = -lgtest_main -lgtest

##################################################################################
# option
#
CFLAGS = -g3 -Wall -D$(UNAME) -D_REENTRANT
# CFLAGS += -Wextra -fPIC -Wl,-rpath=.  -DUSE_PIO -DUSE_DUMMYDEV

##################################################################################
# library & application
SLIB_APP = $(LIB_DIR)/lib$(APP_NAME).a
# LIB_APP = $(LIB_DIR)/lib$(APP_NAME).so
EXE_APP  = $(BIN_DIR)/$(APP_NAME)
TEST_APP = $(BIN_DIR)/$(TEST_APP_NAME)
#
ALL_SRC  = $(wildcard $(addsuffix /*.cpp, $(SRC_DIR)))
EXE_SRC  = $(SRC_DIR)/main.cpp
LIB_SRC  = $(filter-out $(EXE_SRC), $(ALL_SRC))
LIB_SRC += $(wildcard $(SRC_UTIL_DIR)/*.cpp)
TEST_SRC = $(wildcard $(TEST_SRC_DIR)/*.cpp) $(wildcard $(TEST_SRC_UTIL_DIR)/*.cpp)
#
LIB_OBJS=$(LIB_SRC:%.cpp=%.o)
EXE_OBJS =$(EXE_SRC:%.cpp=%.o)
TEST_OBJS=$(TEST_SRC:%.cpp=%.o)

##################################################################################
# Target
COMPILE_TARGETS = $(SLIB_APP) $(LIB_APP) $(EXE_APP) $(TEST_APP)
all: compile_title $(COMPILE_TARGETS)
compile_title:
	@echo
	@echo "COMPILE_TARGETS="$(COMPILE_TARGETS)"\n"
	@echo ---- $(MAKE) $(COMPILE_TARGETS) "("$(shell basename $(shell pwd))")" ----- "\n"


# separate compile -- make staic library
$(SLIB_APP): $(LIB_OBJS)
	@echo "\n\n  "$^" --> "$@"\n"
	@if [ ! -d $(LIB_DIR) ]; then \
		mkdir -p $(LIB_DIR); \
	fi
	ar rcs $@ $^
# @rm $(LIB_OBJS)

# separate compile -- make shared library
$(LIB_APP): $(LIB_OBJS)
	@echo "\n\n  "$^" --> "$@"\n"
	@if [ ! -d $(LIB_DIR) ]; then \
		mkdir -p $(LIB_DIR); \
	fi
	$(eval LIB_CFLAGS = $(CFLAGS) -fPIC)
	$(CXX) -shared -o $@ $^ $(LIB_CFLAGS) $(LINK_DIRS) $(LINK)
# @rm $(LIB_OBJS)

# separated compile -- make executing application
$(EXE_APP): $(EXE_OBJS) $(SLIB_APP)
	@echo "\n\n  "$^" --> "$@"\n"
	@if [ ! -d $(BIN_DIR) ]; then \
		mkdir -p $(BIN_DIR); \
	fi
	$(CXX) -o $@ $^ $(CFLAGS) $(LINK_DIRS) $(LINK)
# @rm $(EXE_OBJS)

# separated compile -- make unit_test application
$(TEST_APP): $(TEST_OBJS) $(SLIB_APP)
	@echo "\n\n  "$^" --> "$@"\n"
	@if [ ! -d $(BIN_DIR) ]; then \
		mkdir -p $(BIN_DIR); \
	fi
	$(CXX) -o $@ $^ $(CFLAGS) $(LINK_DIRS) $(LINK_GTEST) $(LINK)
# @rm $(TEST_OBJS)

### common compile -- make object file
%.o: %.cpp
	@echo "\n\n  "$<" --> "$@"\n"
	@if [ "$(LIB_APP)" != "" ]; then \
		echo "$(CXX) -c $(CFLAGS) -fPIC $(INCLUDES_PATH) -o $@ $<"; \
		$(CXX) -c $(CFLAGS) -fPIC $(INCLUDES_PATH) -o $@ $<;\
	else \
		echo "$(CXX) -c $(CFLAGS) $(INCLUDES_PATH) -o $@ $<"; \
		$(CXX) -c $(CFLAGS) $(INCLUDES_PATH) -o $@ $<; \
	fi

# make clean
clean:
	rm -f $(LIB_OBJS)
	rm -f $(EXE_OBJS)
	rm -f $(TEST_OBJS)
	rm -f *~ core
	rm -f $(INCLUDE_DIR)/*~
	rm -f $(INCLUDE_UTIL_DIR)/*~
	rm -f $(SRC_DIR)/*~
	rm -f $(SRC_UTIL_DIR)/*~

# make cleanall
cleanall: clean
	rm -f $(SLIB_APP)
	rm -f $(EXE_APP)
	rm -f $(TEST_APP)
