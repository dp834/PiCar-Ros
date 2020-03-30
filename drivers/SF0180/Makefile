
src = SF0180.c
obj = SF0180.o
TEST_SRC = test.c
TEST_OBJ = SF0180.o $(realpath ../PCA9685/PCA9685.o)
TEST_EXE = servo_test

build: $(obj)

test: $(obj)
	$(CC) $(CFLAGS) $(TEST_SRC) $(TEST_OBJ) -I$(INCLUDE) -o $(TEST_EXE)
	cp $(TEST_EXE) $(INSTALL)

%.o: %.c
	$(CC) $(CFLAGS) $< -c -I $(INCLUDE)

clean:
	-@rm $(obj) $(TEST_EXE) 2> /dev/null
