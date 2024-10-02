#include "utilities.h"
#include <stdio.h>

volatile enum mode curr_mode;

volatile enum mode get_mode(void){
	return curr_mode;
}

void set_mode(enum mode input){
	curr_mode = input;
}

