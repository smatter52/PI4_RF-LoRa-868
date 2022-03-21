# Make file for RPI version of lora library

FILES = lora.o spi.o
CFLAGS = -c -Werror -fmax-errors=2
AFLAGS = -cvr
lora.a: $(FILES)   

.c.o:
	gcc $(CFLAGS) $*.c
	ar $(AFLAGS) lora.a $*.o

.s.o:
	as  $*.s -o $*.o
	ar $(AFLAGS) lora.a $*.o
