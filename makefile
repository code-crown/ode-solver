# compiler and flags
cc = gcc
cflags = -Wall -Wextra -std=c99 -O3 -fpic -Isrc

# directories and files
src_dir = src
obj_dir = obj
bin_dir = bin
examples_dir = examples

# automatically find all library .c files in src directory
src = $(wildcard $(src_dir)/*.c)
obj = $(patsubst $(src_dir)/%.c, $(obj_dir)/%.o, $(src))

# target library
lib = $(bin_dir)/libodeslv.so

# default target: build only the library
all: $(lib)

# create directories
$(obj_dir):
	mkdir -p $(obj_dir)

$(bin_dir):
	mkdir -p $(bin_dir)

# compile each library .c file to a .o file
$(obj_dir)/%.o: $(src_dir)/%.c | $(obj_dir)
	$(cc) $(cflags) -c $< -o $@

# create shared library (.so)
$(lib): $(obj) | $(bin_dir)
	$(cc) -shared -o $@ $^

# --- explicit rules for examples ---
# usage: type "make main" or "make xy" (if file is examples/xy.c)
%: $(examples_dir)/%.c $(lib) | $(bin_dir)
	$(cc) $(cflags) $< -L$(bin_dir) -lodeslv -Wl,-rpath,'$$ORIGIN' -o $(bin_dir)/$@

# clean up build files
clean:
	rm -rf $(obj_dir) $(bin_dir)

.phony: all clean