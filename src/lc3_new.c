#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
/* unix only */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

#define true 1
#define false 0

#define PC_START 0x3000
#define MEMORY_MAX (1 << 16)    //lc-3 memory has 65.536 memory locations
uint16_t memory[MEMORY_MAX];

/* -- Registers -- :
    - is a slot for storing a single value on the CPU
    - for a CPU to work with a piece of data, it has to be in one of the registers
    - since there are just a few registers, only a minimal amount of data can be loaded at any given time
    - programs loads values from memory into registers, calculate those values into other registers, then stores the final result back in memory

    R0...R7: general purpose registers (perform any program calculations)
    PC: Instruction pointer. Address of the next instruction in memory to execute
    R_COND: Tells us information about previous calculation. (conditional flag register)
*/
typedef enum {
    R0 = 0,
    R1 = 1,
    R2 = 2,
    R3 = 3,
    R4 = 4,
    R5 = 5,
    R6 = 6,
    R7 = 7,
    PC = 8,
    R_COND = 9,
    NUM_OF_REGISTERS = 10
}Reg;
uint16_t registers[NUM_OF_REGISTERS];   //10 registers in total, each one is 16 bits

/* -- Memory Mapped Registers -- */
typedef enum {
    KBSR = 0xFE00,  // keyboard status
    KBDR = 0xFE02   // keyboard data
}MMR;

/* -- Condition flags -- */
typedef enum {
    FL_POS = 1 << 0, // P
    FL_ZRO = 1 << 1, // Z
    FL_NEG = 1 << 2  // N
} Cond_Flag;

/* -- Instruction set -- :
    - An instruction is a command which tells the CPU to do some task, such as add two numbers. 
    - Instructions have OPCODE(indicates the kind of task to perform) and PARAMETERS(provide inputs to the task).
*/
typedef enum {
    OP_BR = 0,	// branch
    OP_ADD,		// add
    OP_LD,		// load
    OP_STORE,	// store
    OP_JSR,		// jump register
    OP_AND,		// bitwise AND
    OP_LDR,		// load register
    OP_STR,		// store register
    OP_RTI,		// unused
    OP_NOT,     // bitwise NOT
    OP_LDI,		// load indirect
    OP_STI,		// store indirect
    OP_JMP,		// jump
    OP_RES,		// reserved (unused)
    OP_LEA,		// load effective address
    OP_TRAP		// execute trap
} Opcodes;

/* -- Trap Codes -- */
typedef enum {
    TRAP_GETC  = 0x20,
    TRAP_OUT   = 0x21,
    TRAP_PUTS  = 0x22,
    TRAP_IN    = 0x23,
    TRAP_PUTSP = 0x24,
    TRAP_HALT  = 0x25
}Trap;

/* ----- Input buffering ----- */
struct termios original_tio;
void disable_input_buffering() {
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
	tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
	struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
	return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

/* ----- handle interrupt ----- */
void handle_interrupt(int signal) {
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

/* ----- sign extend v1 and v2 ----- */
uint16_t sign_extend(uint16_t x, int bit_count) {
    if ((x >> (bit_count - 1)) & 1) {
        x = x | (0xFFFF << bit_count);
    }   

    return x;
}

uint16_t my_sign_extend(uint16_t imm) {
	imm &= 0b0000000000011111;
	if (imm & 0b10000) {
		imm |= 0b0000000000011111;
	}

	return imm;
}

/* ----- swap ----- */
uint16_t swap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

/* ----- update flags ----- */
void update_flags(uint16_t reg) {
    if (registers[reg] == 0) {
        registers[R_COND] = FL_ZRO;
    } else if (registers[reg] >> 15) { //if the 15-bit is true (1), it means it is negative
        registers[R_COND] = FL_NEG;

    } else {
        registers[R_COND] = FL_POS;
    }
}

/* ----- Read image file ----- */
void read_image_file(FILE *file) {  //reading an LC-3 program into memory
    uint16_t origin; //origin tells us where in memory to place the image
    fread(&origin, sizeof(origin), 1, file);  //reads data from the given stream into the array pointed to by origin 
    origin = swap16(origin);

    uint16_t max_read = MEMORY_MAX - origin;    
    uint16_t *p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    while ((read--) > 0) {
        *p = swap16(*p);
        ++p;
    }
}

int read_image(const char *image_path) {
    FILE *file = fopen(image_path, "rb");
    if (!file) {
        return 0;
    }
    read_image_file(file);
    fclose(file);
    
    return 1;
}

/* ----- Memory access ----- */
void mem_write(uint16_t address, uint16_t val) {
    memory[address] = val;
}

uint16_t mem_read(uint16_t address) {
    if (address == KBSR) {
        if (check_key()) {
            memory[KBSR] = (1 << 15);
            memory[KBDR] = getchar();
        } else {
            memory[KBSR] = 0;
        }
    }

    return memory[address];
}

/* ----- Trap functions ----- */
void trap_getc() {
    char c = getchar();
    registers[R0] = (uint16_t)c;
    update_flags(R0);
}

void trap_out() {
	putc((char)registers[R0], stdout);
    fflush(stdout);
}

void trap_puts() {
	uint16_t *c = memory + registers[R0];
	while (*c) {
		putc((char)*c, stdout);
		++c;
	}
	fflush(stdout);
}

void trap_in() {
	printf("Write a single character: \n");
	char c = getchar();
	putc(c, stdout);
	fflush(stdout);
	registers[R0] = (uint16_t)c;
	update_flags(R0);
} 

void trap_putsp() {
	uint16_t *c = memory + registers[R0];
	
	while (*c) {
		char c1 = (*c) & 0xFF;
		putc(c1, stdout);
		char c2 = ((*c) >> 8) & 0xFF;
		if (c2) {
			putc(c2, stdout);
		}
		++c;
	}
	fflush(stdout);
}

void trap_halt(bool running) {
	printf("Program Halted!");
	fflush(stdout);
	running = false;
}

/* main */
int main(int argc, const char* argv[]) {
	// setup
	signal(SIGINT, handle_interrupt);
	disable_input_buffering();	
    
    // load arguments
	if (argc < 2) {
		printf("lc3 [image-file] ...\n");
		exit(2);
	}

	for (int j = 1; j < argc; j++) {
		if (!read_image(argv[j])) {
			printf("Failed to load image: %s\n", argv[1]);
			exit(1);
		}
	}

	registers[R_COND] = FL_ZRO; // enum {PC_START = 0x3000};

	/* Setting PC to the starting position (0x3000 by default) */
	registers[PC] = PC_START;	

	bool running = true;
	while (running) {
		uint16_t instruction = mem_read(registers[PC]++);
		uint16_t opcode = instruction >> 12;	//shifts 15...12 (15,14,13,12). It's holding 4 bits, which are the opcode

		switch (opcode) {
			case OP_ADD: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;     //takes only 3 bits (11,10,9) and stores in a 16-bit variable
                uint16_t sr1_reg = (instruction >> 6) & 0x7;
                
                if ((instruction >> 5) & 0x1) { //immediate mode - bit[5] = 1
                    uint16_t imm5 = my_sign_extend(instruction & 0x1F);
                    registers[dr_reg] = registers[sr1_reg] + imm5;   //so this actually takes from the general register that corresponds with the sr1_reg value position. Ex: inside the instruction, on SR1 bits, we can find a position number, such as 1, which represents our R1 general purpose register.
                                                                     //Inside this R1 register is actually the number that we want to add read from the memory. That is why we get and store the value on register
                } else { //register mode
                    uint16_t sr2_reg = instruction & 0x7;
                    registers[dr_reg] = registers[sr1_reg] + registers[sr2_reg];
                }
                update_flags(dr_reg);
                break;
            }

			case OP_AND: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t sr1_reg = (instruction >> 6) & 0x7;

                if ((instruction >> 5) & 0x1) { //immediate mode - bit[5] = 1
                    uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
                    registers[dr_reg] = registers[sr1_reg] & imm5;
                } else {    //register mode
                    uint16_t sr2_reg = (instruction & 0x7);
                    registers[dr_reg] = registers[sr1_reg] & registers[sr2_reg];
                }
                update_flags(dr_reg);
                break;
            }

            case OP_NOT: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t sr_reg = (instruction >> 6) & 0x7;

                registers[dr_reg] = ~(registers[sr_reg]);
                update_flags(dr_reg);
                break;
            }

            case OP_BR: {
                uint16_t nzp = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);

                if (nzp & registers[R_COND]) {
                    registers[PC] += pc_offset9;
                } 
                break;
            }

            case OP_JMP: {
                uint16_t base_r = (instruction >> 6) & 0x7;
                registers[PC] = registers[base_r];
                break;
            }

            case OP_JSR: {
                uint16_t bit_11 = (instruction >> 11) & 0x1;
                registers[R7] = registers[PC];

                if (!bit_11) {  //JSRR - bit[11] = 0
                    uint16_t base_reg = (instruction >> 6) & 0x7;
                    registers[PC] = registers[base_reg];
                } else {
                    uint16_t pc_offset11 = sign_extend(instruction & 0x7FF, 11);
                    registers[PC] += pc_offset11;
                }
                break;
            }

            case OP_LD: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);

                registers[dr_reg] = mem_read(registers[PC] + pc_offset9);
                update_flags(dr_reg);
                break;
            }

            case OP_LDI: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);
                
                registers[dr_reg] = mem_read(mem_read(registers[PC] + pc_offset9));
                update_flags(dr_reg);
                break;
            }

            case OP_LDR: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t offset6 = sign_extend(instruction & 0x3F, 6);
                uint16_t base_reg = (instruction >> 6) & 0x7;

                registers[dr_reg] = mem_read(registers[base_reg] + offset6);
                update_flags(dr_reg);
                break;
            }

            case OP_LEA: {
                uint16_t dr_reg = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);

                registers[dr_reg] = registers[PC] + pc_offset9; //there is no mem_read, because the dr_reg is storing the address instead of the value
                update_flags(dr_reg);
                break;
            }

            case OP_STORE: {
                uint16_t sr = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);

                mem_write(registers[PC] + pc_offset9, registers[sr]);
                break;
            }

            case OP_STI: {
                uint16_t sr = (instruction >> 9) & 0x7;
                uint16_t pc_offset9 = sign_extend(instruction & 0x1FF, 9);

                mem_write(mem_read(registers[PC] + pc_offset9), registers[sr]);
                break;
            }

            case OP_STR: {
                uint16_t sr = (instruction >> 9) & 0x7;
                uint16_t base_reg = (instruction >> 6) & 0x7;
                uint16_t offset6 = sign_extend(instruction & 0x3F, 6);

                mem_write(registers[base_reg] + offset6, registers[sr]);
                break;
            }

            case OP_TRAP: {
                uint16_t trap_vect8 = sign_extend(instruction & 0xFF, 9);
                registers[R7] = registers[PC]; //save the last address so that PC can go back to the same spot as before
                registers[PC] = mem_read(trap_vect8);
                
                /* -- Trap codes --*/
                switch (trap_vect8) {
                    case TRAP_GETC: {
						trap_getc();
                        break;
                    }
                    case TRAP_OUT: {
                        trap_out();
						break;
                    }
                    case TRAP_PUTS: { // output a null-terminated string. Write a string of characters to the console
						trap_puts();
                        break;
                    }
                    case TRAP_IN: {
						trap_in();
                        break;
                    }
                    case TRAP_PUTSP: {
						trap_putsp();
                        break;
                    }
                    case TRAP_HALT: {
						trap_halt(running);
                        break;
                    }
                }
                break;
            }
		}
	}
	restore_input_buffering();
}
