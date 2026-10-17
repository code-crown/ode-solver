# compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O3 -Isrc

# directories
SRC = src
OBJ = obj
EXMPL = examples
BIN = bin

# os detection for file extension and directory creation
ifeq ($(OS),Windows_NT)
    MKDIR_OBJ = if not exist $(OBJ) mkdir $(OBJ)
    MKDIR_BIN = if not exist $(BIN) mkdir $(BIN)
    LIB_EXT = dll
    LIB_LDFLAGS = -L$(BIN) -lodeslv
else
    MKDIR_OBJ = mkdir -p $(OBJ)
    MKDIR_BIN = mkdir -p $(BIN)
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        LIB_EXT = dylib
        LIB_LDFLAGS = -L$(BIN) -lodeslv -Wl,-rpath,@executable_path/$(BIN)
    else
        LIB_EXT = so
        LIB_LDFLAGS = -L$(BIN) -lodeslv -Wl,-rpath,'$$ORIGIN/$(BIN)'
    endif
endif

# name of the final shared library
TARGET = $(BIN)/libodeslv.$(LIB_EXT)

# standard target
all: $(TARGET)

# create object file for shared library
$(OBJ)/odeslv.o: $(SRC)/odeslv.c
	@$(MKDIR_OBJ)
	$(CC) $(CFLAGS) -fpic -c $< -o $@

$(TARGET): $(OBJ)/odeslv.o
	@$(MKDIR_BIN)
	$(CC) -shared $< -o $@


# build example programs using the shared library
# dynamic target: make <name> (searches in examples/<name>.c)
%: $(BIN)/%
	@

# rule: builds the example program and ensures the library exists first
$(BIN)/%: $(EXMPL)/%.c $(TARGET)
	@$(MKDIR_BIN)
	$(CC) $(CFLAGS) $< $(LIB_LDFLAGS) -o $@

# clean build artifacts
clean:
	rm -rf obj bin

.PHONY: all clean