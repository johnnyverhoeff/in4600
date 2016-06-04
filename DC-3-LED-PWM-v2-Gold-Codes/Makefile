CC=g++
CFLAGS=-lstdc++ -lm

ODIR=.

LIBS=-lpigpio
LIBS+=-lrt
LIBS+= -lpthread

_OBJ = main.o 
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

output: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ output 
