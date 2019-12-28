
src =
obj =
TEST_SRC =
TEST_OBJ =
TEST_EXE =

build: $(obj)

test: $(obj)
	$(CC) $(CFLAGS) $(TEST_SRC) $(TEST_OBJ) -I$(INCLUDE) -o $(TEST_EXE)

%.o: %.c
	$(CC) $(CFLAGS) $< -c

clean:
	-@rm $(obj) 2> /dev/null
