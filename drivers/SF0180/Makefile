
src = SF0180.c
obj = SF0180.o

build: $(obj)

%.o: %.c
	$(CC) $(CFLAGS) $< -c

clean:
	-@rm $(obj) 2> /dev/null
