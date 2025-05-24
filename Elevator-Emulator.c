/*
 * Elevator-Emulator.c
 *
 * Main file
 *
 * Authors: Peter Sutton, Ahmed Baig
 * Modified by Yiyang Yu
 */ 

/* Definitions */

#define F_CPU 8000000L
// Define speed switching details
#define FAST_SPEED 100
#define SLOW_SPEED 300
#define SPEED_SWITCH 7
#define SSD_A 4
#define SSD_D 5
#define SSD_G 6

/* External Library Includes */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>

/* Internal Library Includes */

#include "display.h"
#include "ledmatrix.h"
#include "buttons.h"
#include "serialio.h"
#include "terminalio.h"
#include "timer0.h"

/* Data Structures */

typedef enum {UNDEF_FLOOR = -1, FLOOR_0=0, FLOOR_1=4, FLOOR_2=8, FLOOR_3=12} ElevatorFloor;

/* Global Variables */
uint32_t time_since_move;
ElevatorFloor current_position;
ElevatorFloor destination;
ElevatorFloor traveller_floor;
// For traveller status determine
bool traveller_active;
bool traveller_moving;

/* Internal Function Declarations */

void initialise_hardware(void);
void start_screen(void);
void start_elevator_emulator(void);
void handle_inputs(void);
void draw_elevator(void);
void draw_floors(void);
void draw_traveller(void);
void display_terminal_info(uint8_t current_position, uint8_t destination);
uint16_t get_speed(void);
void direction_ssd(ElevatorFloor current_position, ElevatorFloor destination);

/* Main */

int main(void) {
	// Setup hardware and call backs. This will turn on 
	// interrupts.
	initialise_hardware();
	
	// Show the splash screen message. Returns when display is complete
	start_screen();
	
	// Start elevator controller software
	start_elevator_emulator();
}

/* Internal Function Definitions */

/**
 * @brief All hardware initialisation occurs here
 * @arg none
 * @retval none
*/
void initialise_hardware(void) {
	
	ledmatrix_setup();
	init_button_interrupts();
	// Setup serial port for 19200 baud communication with no echo
	// of incoming characters
	init_serial_stdio(19200,0);
	
	init_timer0();
	
	// Turn on global interrupts
	sei();

	// Set PortC Pin 7 as input for S2
	DDRC &= ~(1 << SPEED_SWITCH);
	PORTC |= (1 << SPEED_SWITCH);

	//Set PortC Pin 4-6 as outputs for SSD segments
	DDRC |= (1 << SSD_A) | (1 << SSD_D) |(1 << SSD_G);
	PORTC |= ~((1 << SSD_A) | (1 << SSD_D) |(1 << SSD_G));
}

/**
 * @brief Displays the "EC" start screen with elevator symbol
 * @arg none
 * @retval none
*/
void start_screen(void) {
	// Clear terminal screen and output a message
	clear_terminal();
	move_terminal_cursor(10,10);
	printf_P(PSTR("Elevator Controller"));
	move_terminal_cursor(10,12);
	printf_P(PSTR("CSSE2010/7201 project by Yiyang Yu 48758004"));
	
	// Show start screen
	start_display();
	
	// Animation variables
	uint32_t doors_frame_time = 0;
	uint32_t interval_delay = 150;
	uint8_t frame = 0;
	uint8_t doors_opening_closing = 1; // 1 => opening, 0 => closing
	
	
	// Wait until a button is pressed, or 's' is pressed on the terminal
	while(1) {
		
		// Don't worry about this if/else tree. Its purely for animating
		// the elevator doors on the start screen
		if (get_current_time() - doors_frame_time  > interval_delay) {
			start_display_animation(frame);
			doors_frame_time   = get_current_time(); // Reset delay until next movement update
			if (doors_opening_closing) {
				interval_delay = 150;
				frame++;
				if (frame == 1) interval_delay = 2000;
				if (frame == 3) doors_opening_closing = 0;
			} else {
				interval_delay = 150;
				frame--;
				if (frame == 2) interval_delay = 500;
				if (frame == 0) doors_opening_closing = 1;
			}
		}
	
		// First check for if a 's' is pressed
		// There are two steps to this
		// 1) collect any serial input (if available)
		// 2) check if the input is equal to the character 's'
		char serial_input = -1;
		if (serial_input_available()) {
			serial_input = fgetc(stdin);
		}
		// If the serial input is 's', then exit the start screen
		if (serial_input == 's' || serial_input == 'S') {
			break;
		}
		// Next check for any button presses
		int8_t btn = button_pushed();
		if (btn != NO_BUTTON_PUSHED) {
			break;
		}
	}
}

/**
 * @brief Initialises LED matrix and then starts infinite loop handling elevator
 * @arg none
 * @retval none
*/
void start_elevator_emulator(void) {
	
	// Clear the serial terminal
	clear_terminal();
	
	// Initialise Display
	initialise_display();
	
	// Clear a button push or serial input if any are waiting
	// (The cast to void means the return value is ignored.)
	(void)button_pushed();
	clear_serial_input_buffer();

	// Initialise local variables
	time_since_move = get_current_time();
	
	// Draw the floors and elevator
	draw_elevator();
	draw_floors();
	
	current_position = FLOOR_0;
	destination = FLOOR_0;
	
	while(true) {
		
		// Only update the elevator every 200 ms
		if (get_current_time() - time_since_move > get_speed()) {	
			
			// Adjust the elevator based on where it needs to go
			if (destination - current_position > 0) { // Move up
				current_position++;
			} else if (destination - current_position < 0) { // Move down
				current_position--;
			}

			// Determine the status of traveller
			if (traveller_active && current_position == traveller_floor) {
				traveller_active = false;
				traveller_moving = true;
				
				// Clear the LED
				int8_t y = traveller_floor +1;
				update_square_colour(4, y, MATRIX_COLOUR_EMPTY);
				
				// Move traveller to floor 0
				destination = FLOOR_0;
			}
			if (traveller_moving && current_position == FLOOR_0) {
				traveller_moving = false;
			}
			
			direction_ssd(current_position, destination);

			// As we have potentially changed the elevator position, lets redraw it
			draw_elevator();

			draw_traveller();
			
			time_since_move = get_current_time(); // Reset delay until next movement update
		}
		
		// Handle any button or key inputs
		handle_inputs();

		// Update the terminal info if needed
		display_terminal_info(current_position, destination);

		// Update the ssd display if needed
		direction_ssd(current_position, destination);
	}
}

/**
 * @brief Draws 4 lines of "FLOOR" coloured pixels
 * @arg none
 * @retval none
*/
void draw_floors(void) {
	for (uint8_t i = 0; i < WIDTH; i++) {
		update_square_colour(i, FLOOR_0, FLOOR);
		update_square_colour(i, FLOOR_1, FLOOR);
		update_square_colour(i, FLOOR_2, FLOOR);
		update_square_colour(i, FLOOR_3, FLOOR);
	}
}

/**
 * @brief Draws the elevator at the current_position
 * @arg none
 * @retval none
*/
void draw_elevator(void) {
	
	// Store where it used to be with old_position
	static uint8_t old_position; // static variables maintain their value, every time the function is called
	
	int8_t y = 0; // Height position to draw elevator (i.e. y axis)
	
	// Clear where the elevator was
	if (old_position > current_position) { // Elevator going down - clear above
		y = old_position + 3;
		} else if (old_position < current_position) { // Elevator going up - clear below
		y = old_position + 1;
	}
	if (y % 4 != 0) { // Do not draw over the floor's LEDs
		update_square_colour(1, y, EMPTY_SQUARE);
		update_square_colour(2, y, EMPTY_SQUARE);
	}
	old_position = current_position;
	
	// Draw a 2x3 block representing the elevator
	for (uint8_t i = 1; i <= 3; i++) { // 3 is the height of the elevator sprite on the LED matrix
		y = current_position + i; // Adds current floor position to i=1->3 to draw elevator as 3-high block
		if (y % 4 != 0) { // Do not draw on the floor
			update_square_colour(1, y, ELEVATOR);
			update_square_colour(2, y, ELEVATOR); // Elevator is 2 LEDs wide so draw twice
		}
	}
}

/**
 * @brief Reads btn values and serial input and adds a traveller as appropriate
 * @arg none
 * @retval none
*/
void handle_inputs(void) {
	
	/* ******** START HERE ********
	
	 The following code handles moving the elevator using the buttons on the
	 IO Board. Add code to handle BUTTON2_PUSHED and BUTTON3_PUSHED
	 
	 Here is how the following code works:
	 1. Get btn presses (if any have occurred). Remember that this is
		all handled in the buttons.c/h library.
	 2. Use an if/else tree based on which of the buttons has been
		pressed.
	 3. Set the destination of the elevator to the FLOOR_X corresponding
		with the particular button that was pressed.
	
	*/
	
	// We need to check if any button has been pushed
	uint8_t btn = button_pushed();

	// Collect any serial input
	char serial_input = -1;
	if (serial_input_available()) {
		serial_input = fgetc(stdin);
	}
	
	// Judge the button/key input and traveller status to set destination
	if (!traveller_active && !traveller_moving) {
		if (btn == BUTTON0_PUSHED || serial_input == '0') {
			traveller_floor = FLOOR_0;
		} else if (btn == BUTTON1_PUSHED || serial_input == '1') {
			traveller_floor = FLOOR_1;
		} else if (btn == BUTTON2_PUSHED || serial_input == '2') {
			traveller_floor = FLOOR_2;
		} else if (btn == BUTTON3_PUSHED || serial_input == '3') {
			traveller_floor = FLOOR_3;
		} else {
			return; // No button/key pressed
		}
	
		// Create the traveller
		traveller_active = true;
		destination = traveller_floor;
	}
	
}


// Define a variable and a string to handle the comparison afterward
static int previous_position = -1;
static char previous_direction[11] = "";
// Called to display infos in serial terminal (putty)
void display_terminal_info(uint8_t current_position, uint8_t destination) {
	// Set a pointer for strings refering later
	const char *direction;

	// Compare current position to determine the direction
	if (current_position < destination) {
		direction = "Up";
	}  else if (current_position > destination) {
		direction = "Down";
	} else {
		direction = "Stationary";
	}

	// Convert the matrix position into floor number
	int floor_number = ((int)current_position) / 4;

	if (current_position != previous_position || strcmp(previous_direction, direction) != 0) { // Compare the current info with previous one to see if update needed
		move_terminal_cursor(1, 1);  // Allocate the infos at the correct place
		printf_P(PSTR("Current Floor: %d   "), floor_number);  // Put some space after to overwrite the previous printing

		move_terminal_cursor(1, 2);  // Print the direction info at next line
		printf_P(PSTR("Direction: %s        "), direction);

		// Save the current infos for comparison
		strncpy(previous_direction, direction, sizeof(previous_direction));
		previous_position = current_position;
	}
}

// Called to draw the traveller
void draw_traveller(void) {
	if (traveller_active) {
		int8_t y = traveller_floor + 1;
		update_square_colour(4, y, MATRIX_COLOUR_TRAVELLER_0); // Draw as light red
	}
}

// Called for speed switch
uint16_t get_speed(void) {
	if ((PINC & (1 << SPEED_SWITCH)) == 0) { // Use bit masking to judge if the switch is 0/1
		return SLOW_SPEED;
	} else {
		return FAST_SPEED;
	}
}


void direction_ssd(ElevatorFloor current_position, ElevatorFloor destination) {

	PORTC &= ~((1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G));

	if (destination > current_position) {
		PORTC |= (1 << SSD_A);
	} else if (destination < current_position) {
		PORTC |= (1 << SSD_D);
	} else {
		PORTC |= (1 << SSD_G);
	}
}