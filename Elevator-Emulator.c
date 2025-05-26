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
#define SPEED_SWITCH PC7
// Define SSD connected pins
#define SSD_A PC4
#define SSD_B PD2
#define SSD_C PD3
#define SSD_D PC5
#define SSD_E PD5
#define SSD_F PD4
#define SSD_G PC6
#define SSD_CC PD1
#define SSD_DP PD0
// Define destination swithes
#define SWITCH_S0 PC2
#define SWITCH_S1 PC3
// Define buzzer
#define BUZZER PD7

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
ElevatorFloor potential_destination;
// Define a variable and a string to handle the comparison afterward
int previous_position = -1;
char previous_direction[11] = "";
// For traveller status determine
bool traveller_active;
bool traveller_moving;
// Traveller colour
uint8_t traveller_destination;
// To toggle the SSD
bool show_ssd_left = true;
uint32_t time_since_ssd_toggle;
// To count the floors traveled
uint8_t floors_with_traveller = 0;
uint8_t floors_without_traveller = 0;
int8_t previous_floor = 0;

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
uint8_t switch_destination(void);
uint8_t get_traveller_destination(uint8_t destination);
void toggle_ssd(void);
void update_floor_num(void);

/* Main */

int main(void) {
	// Setup hardware and call backs. This will turn on 
	// interrupts.
	initialise_hardware();
	
	// Show the splash screen message. Returns when display is complete
	start_screen();
	
	// Initialise the timer
	init_timer0();

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

	// Set PortC Pins 0,1,4-6 as outputs for SSD segments
	DDRC |= (1 << SSD_A) | (1 << SSD_D) |(1 << SSD_G) | (1 << SSD_CC) | (1 << SSD_DP);
	PORTC &= ~((1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G) | (1 << SSD_CC) | (1 << SSD_DP));

	// Set PortD Pins 4-7 as outputs for SSD segments
	DDRD |= (1 << SSD_B) | (1 << SSD_C) | (1 << SSD_E) | (1 << SSD_F);
	PORTD &= ~((1 << SSD_B) | (1 << SSD_C) | (1 << SSD_E) | (1 << SSD_F));

	// Set PortC Pin 2 connect to S0, Pin 3 connect to S1
	DDRC &= ~((1 << SWITCH_S0) | (1 << SWITCH_S1));
	PORTC |= (1 << SWITCH_S0) | (1 << SWITCH_S1);

	// Set CC high at first (left SSD is activated)
	PORTC |= (1 << SSD_CC);
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
	time_since_ssd_toggle = 0;
	
	// Draw the floors and elevator
	draw_elevator();
	draw_floors();
	
	current_position = FLOOR_0;
	destination = FLOOR_0;
	
	while(true) {
		
		// Update the Elevator as selected speed
		if (get_current_time() - time_since_move > get_speed()) {	
			
			// Adjust the elevator based on where it needs to go
			if (destination - current_position > 0) { // Move up
				current_position++;
			} else if (destination - current_position < 0) { // Move down
				current_position--;
			}

			// Update the floor travelled
			update_floor_num();

			// Determine the status of traveller
			if (traveller_active && current_position == traveller_floor) {
				traveller_active = false;
				traveller_moving = true;
				
				// Clear the LED
				int8_t y = traveller_floor +1;
				update_square_colour(4, y, MATRIX_COLOUR_EMPTY);
				
				// Move traveller to destination
				destination = potential_destination;
			}
			if (traveller_moving && current_position == destination) {
				traveller_moving = false;
			}
			
			direction_ssd(current_position, destination);

			// As we have potentially changed the elevator position, lets redraw it
			draw_elevator();
			// Redraw the traveller
			draw_traveller();

			

			time_since_move = get_current_time(); // Reset delay until next movement update
		}

		// Toggle the SSD frequently to show both at the same time
		if (get_current_time() - time_since_ssd_toggle > 0.1) {
			toggle_ssd();
			_delay_ms(2);  // Use delay to let cpu 'wait' for 1ms
			time_since_ssd_toggle = get_current_time();
		}
		
		// Handle any button or key inputs
		handle_inputs();

		// Update the terminal info if needed
		display_terminal_info(current_position, destination);

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
		// Define potential movements
		ElevatorFloor potential_floor;
		uint8_t destination_floor;
		
		// Judge the button/key input
		if (btn == BUTTON0_PUSHED || serial_input == '0') {
			potential_floor = FLOOR_0;
		} else if (btn == BUTTON1_PUSHED || serial_input == '1') {
			potential_floor = FLOOR_1;
		} else if (btn == BUTTON2_PUSHED || serial_input == '2') {
			potential_floor = FLOOR_2;
		} else if (btn == BUTTON3_PUSHED || serial_input == '3') {
			potential_floor = FLOOR_3;
		} else {
			return; // No button/key pressed
		}
		
		// Handle the switch input
		destination_floor = switch_destination();
		potential_destination = destination_floor * 4;
		
		// Judge if the traveller already on his destination, ignore if so
		if (potential_floor == potential_destination) {
			return;
		}

		// Create the traveller
		traveller_active = true;
		traveller_floor = potential_floor;
		destination = traveller_floor;
		traveller_destination = get_traveller_destination(destination_floor); // Used to indicate the colour
	}
	
}

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
	int8_t floor_number = ((int)current_position) / 4;

	if (current_position != previous_position || strcmp(previous_direction, direction) != 0) { // Compare the current info with previous one to see if update needed
		move_terminal_cursor(1, 1);  // Allocate the infos at the correct place
		printf_P(PSTR("Current Floor: %u   "), floor_number);  // Put some space after to overwrite the previous printing

		move_terminal_cursor(1, 2);  // Print the direction info at next line
		printf_P(PSTR("Direction: %s        "), direction);

		// Save the current infos for comparison
		strncpy(previous_direction, direction, sizeof(previous_direction));
		previous_position = current_position;
	}

	/*int8_t previous_floor = 0;
	if (floor_number != previous_floor) {
		if (traveller_moving) {
			floors_with_traveller++;
		} else {
			floors_without_traveller++;
		}
		previous_floor = floor_number;

		move_terminal_cursor(1, 3);
		printf_P(PSTR("Floors with Traveller: %u"), floors_with_traveller);

		move_terminal_cursor(1, 4);
		printf_P(PSTR("Floors without Traveller: %u"), floors_without_traveller);
	}**/
}

// Called to draw the traveller
void draw_traveller(void) {
	if (traveller_active) {
		int8_t y = traveller_floor + 1;
		update_square_colour(4, y, traveller_destination); // Draw as destination indicated
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

// Called for left ssd
void direction_ssd(ElevatorFloor current_position, ElevatorFloor destination) {
	// Clear the segments
	PORTC &= ~((1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G));

	// Judge which segment turns on
	if (destination > current_position) {
		PORTC |= (1 << SSD_A);
	} else if (destination < current_position) {
		PORTC |= (1 << SSD_D);
	} else {
		PORTC |= (1 << SSD_G);
	}
}

// Handle switch input
uint8_t switch_destination(void) {
	uint8_t s0 = (PINC >> SWITCH_S0) & 1;
	uint8_t s1 = (PINC >> SWITCH_S1) & 1;
	return (s1 << 1) | s0;
}

// Get corresponding traveller destination type (as object)
uint8_t get_traveller_destination(uint8_t destination) {
	switch (destination){
		case 0:return TRAVELLER_TO_0;
		case 1:return TRAVELLER_TO_1;
		case 2:return TRAVELLER_TO_2;
		case 3:return TRAVELLER_TO_3;
		default:return TRAVELLER_TO_0;
	}
}

// Set the Port C and Port D digits for corresponding SSD display (0, 1, 2, 3)
const uint8_t portc_digit[4] = {
	(1 << SSD_A) | (1 << SSD_D),
	0,
	(1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G),
	(1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G)
};
const uint8_t portd_digit[4] = {
	(1 << SSD_B) | (1 << SSD_C) | (1 << SSD_E) | (1 << SSD_F),
	(1 << SSD_B) | (1 << SSD_C),
	(1 << SSD_B) | (1 << SSD_E),
	(1 << SSD_B) | (1 << SSD_C)
};

// Called for switching the left and right SSD
void toggle_ssd(void) {
	// Clear the CC
	PORTC &= ~(1 << SSD_CC);
	// Clear the segments
	PORTC &= ~((1 << SSD_A) | (1 << SSD_D) | (1 << SSD_G) | (1 << SSD_DP));
	PORTD &= ~((1 << SSD_B) | (1 << SSD_C) | (1 << SSD_E) | (1 << SSD_F));

	if (show_ssd_left) {
		// Show the direction
		direction_ssd(current_position, destination);
		PORTC |= (1 << SSD_CC);
		PORTC &= ~(1 << SSD_DP);
	} else {
		// Show the floor currently in
		uint8_t floor_num = current_position / 4;
		PORTC |= portc_digit[floor_num];
		PORTD |= portd_digit[floor_num];
		PORTC |= (1 << SSD_DP);
		PORTC &= ~(1 << SSD_CC);
	}
	// Toggle the ssd
	show_ssd_left = !show_ssd_left;
}

// Called for update floor travelling infos
void update_floor_num(void) {
	int8_t current_floor = current_position / 4; // Set for later comparison
	if  (current_floor != previous_floor) {
		if (traveller_moving) { // Judge if the elevator moved any tranveller
			floors_with_traveller++;
		} else {
			floors_without_traveller++;
		}
		previous_floor = current_floor;

		// Placed the info shown in the terminal with an appropriate way
		move_terminal_cursor(1, 3);
		printf_P(PSTR("Floors with Traveller: %u"), floors_with_traveller);
		move_terminal_cursor(1, 4);
		printf_P(PSTR("Floors without Traveller: %u"), floors_without_traveller);
	}
}