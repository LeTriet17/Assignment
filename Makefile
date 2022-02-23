
INC = -Iinclude
LIB = -lpthread

SRC = src
OBJ = obj
INCLUDE = include

CC = gcc
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

vpath %.c $(SRC)
vpath %.h $(INCLUDE)

MAKE = $(CC) $(INC) 

# Object files needed by modules
MEM_OBJ = $(addprefix $(SRC)/, paging.c mem.c cpu.c loader.c)
OS_OBJ = $(addprefix $(SRC)/, mem.c cpu.c loader.c queue.c os.c sched.c timer.c)
SCHED_OBJ = $(addprefix $(SRC)/, cpu.c loader.c mem.c queue.c os.c sched.c timer.c)
HEADER = $(wildcard $(INCLUDE)/*.h)

all: mem sched os test_all

# Just compile memory management modules
mem: $(MEM_OBJ)
	$(MAKE) $(LFLAGS) $(MEM_OBJ) -o mem $(LIB)

# Just compile scheduler
sched: $(SCHED_OBJ)
	$(MAKE) $(LFLAGS) $(SCHED_OBJ) -o os $(LIB)

# Compile the whole OS simulation
os: $(OS_OBJ)
	$(MAKE) $(LFLAGS) $(OS_OBJ) -o os $(LIB)

test_all: test_mem test_sched test_os

test_mem:
	@echo ------ MEMORY MANAGEMENT TEST 0 ------------------------------------
	./mem input/proc/m0
	@echo NOTE: Read file output/m0 to verify your result
	@echo ------ MEMORY MANAGEMENT TEST 1 ------------------------------------
	./mem input/proc/m1
	@echo 'NOTE: Read file output/m1 to verify your result (your implementation should print nothing)'

test_sched:
	@echo ------ SCHEDULING TEST 0 -------------------------------------------
	./os sched_0
	@echo NOTE: Read file output/sched_0 to verify your result
	@echo ------ SCHEDULING TEST 1 -------------------------------------------
	./os sched_1
	@echo NOTE: Read file output/sched_1 to verify your result

test_os:
	@echo ----- OS TEST 0 ----------------------------------------------------
	./os os_0
	@echo NOTE: Read file output/os_0 to verify your result
	@echo ----- OS TEST 1 ----------------------------------------------------
	./os os_1
	@echo NOTE: Read file output/os_1 to verify your result

$(OBJ)/%.o: %.c ${HEADER}
	$(MAKE) $(CFLAGS) $< -o $@

clean:
	rm -f obj/*.o os sched mem



