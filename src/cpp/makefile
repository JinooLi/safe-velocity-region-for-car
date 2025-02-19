TARGET = safe_region_cpp.py
COMPILE_SCRIPT = compile_cpp.sh
OBJS = ./src/safecar.cpp ./include/safecar.hpp
COMPILED = safecar.cpython-310-x86_64-linux-gnu.so


all: $(TARGET) $(COMPILED)

$(COMPILED): $(OBJS)
	echo "Compiling C++ code..."
	./$(COMPILE_SCRIPT)
	echo "C++ code compiled successfully."