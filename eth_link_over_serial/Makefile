#nom de l'executable
EXEC = eth_over_serial

SRC=$(wildcard *.c)
OBJ=$(SRC:.c=.o)

#-----------------------------------------------------------
#compilation :
CC = $(CROSS_COMPILE)gcc
STRIP = $(CROSS_COMPILE)strip
LIBS= -lm -lrt

#option de debugage :
#OPT_Debug= -Wall -g -O2
OPT_Debug= 

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.
# F = 200/(DDMP_FIFO_RATE + 1)
#the program is not fast enough to read DMP at 200Hz => DDMP_FIFO_RATE = 1

#CXXFLAGS = -std=c++11 
#CFLAGS += -Wall -Wextra -pedantic -std=c99

ALL:$(EXEC)

$(EXEC):$(OBJ)
	$(CC) $(CFLAGS) $(OPT_Debug) -o $@ $^ $(LIBS)
	$(STRIP) $@

%.o: %.c
	$(CC) $(CFLAGS) $(OPT_Debug) -o $@ -c $^ $(LIBS)

clean:
	rm -rf *.o *~ *.mod
	rm -rf $(EXEC)
