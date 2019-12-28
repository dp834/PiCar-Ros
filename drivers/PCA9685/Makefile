
src = PCA9685.c
obj = PCA9685.o
TEST_SRC = test.c
TEST_OBJ = PCA9685.o
TEST_EXE = pwm_test

build: $(obj)

test: $(obj)
	$(CC) $(CFLAGS) $(TEST_SRC) $(TEST_OBJ) -I$(INCLUDE) -o $(TEST_EXE)

%.o: %.c
	$(CC) $(CFLAGS) $< -c

clean:
	-@rm $(obj) 2> /dev/null
