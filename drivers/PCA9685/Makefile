
src = PCA9685.c
obj = PCA9685.o

build: $(obj)

%.o: %.c
	$(CC) $(CFLAGS) $< -c

clean:
	-@rm $(obj) 2> /dev/null
