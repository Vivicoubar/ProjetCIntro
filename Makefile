
CFLAGS := -fPIC

all: CFLAGS += -DDEBUG -g
all: libparticle

# Link rules
libparticle: Context.o Particle.o Constraint.o Vec2.o IntArrayGrid.o
	gcc -shared Context.o Particle.o Constraint.o Vec2.o IntArrayGrid.o -o libparticle.so
    
# Compile rules
Vec2.o: Vec2.c Vec2.h
	gcc $(CFLAGS) -c Vec2.c -o Vec2.o
    
IntArrayGrid.o: IntArrayGrid.c IntArrayGrid.h
	gcc $(CFLAGS) -c IntArrayGrid.c -o IntArrayGrid.o
    
Constraint.o: Constraint.c Constraint.h Vec2.h Particle.h
	gcc $(CFLAGS) -c Constraint.c -o Constraint.o
	
Particle.o: Particle.c Particle.h Vec2.h
	gcc $(CFLAGS) -c Particle.c -o Particle.o

Context.o: Context.c Context.h Constraint.h Particle.h Vec2.h IntArrayGrid.h
	gcc $(CFLAGS) -c Context.c -o Context.o

clean:
	del *.o
	del *.so

