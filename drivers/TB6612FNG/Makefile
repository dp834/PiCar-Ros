
src = TB6612FNG.c
obj = TB6612FNG.o
TEST_SRC = test.c
TEST_OBJ = TB6612FNG.o $(realpath ../PCA9685/PCA9685.o)
TEST_EXE = motor_controller_test


build: $(obj)

test: $(obj)
	$(CC) $(CFLAGS) $(TEST_SRC) $(TEST_OBJ) -I$(INCLUDE) -o $(TEST_EXE)
	cp $(TEST_EXE) $(INSTALL)

%.o: %.c
	$(CC) $(CFLAGS) -I$(INCLUDE) $< -c

clean:
	-@rm $(obj) 2> /dev/null
