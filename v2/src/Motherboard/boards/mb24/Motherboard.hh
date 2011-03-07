/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef BOARDS_RRMBV12_MOTHERBOARD_HH_
#define BOARDS_RRMBV12_MOTHERBOARD_HH_

//
// This file describes the Motherboard object, which provides interfaces for
// all facilities provided by the motherboard.  The Motherboard is a singleton;
// call Motherboard::getBoard() to get a reference to the board.
//
// The board should be initialized before use or on reset by calling the init()
// method.
//

#include "UART.hh"
#include "StepperInterface.hh"
#include "Types.hh"
#include "PSU.hh"
#include "Configuration.hh"

class Motherboard {
private:
	const static int STEPPERS = STEPPER_COUNT;

	StepperInterface stepper[STEPPERS];
	PSU psu;
	/// Microseconds since board initialization
	volatile micros_t micros;
	/// Private constructor; use the singleton
	Motherboard();

	// LCD support; should move into it's own class eventually
#if HAS_LCD
	void initLCD();
	uint16_t grabToolInfo(int, uint8_t);

	// MOTOR CONTROLS
	bool lcdmotor_keypress(char);
	void insertoverridecommand(uint8_t, uint8_t*, uint8_t);
	void genmovement(int32_t, int32_t, int32_t, int32_t, int32_t, int32_t);
	bool    lcd_mecmode;
	int32_t lcd_motormovetime;
	int32_t lcd_motormovemms;
	int32_t lcd_motorextrude1;
	int32_t lcd_motorextrude2;

	// DATA WE NEED TO TRACK; should evolve into it's own classes probably.
	// HT = Head Temperature, HTT = Time of last check, SHT = Set temp
	uint16_t HT[NUM_TOOLS];
	micros_t HTT[NUM_TOOLS];
	uint16_t SHT[NUM_TOOLS];
	// PT = same as above for platform
	uint16_t PT;
	micros_t PTT;
	uint16_t SPT;

	// LCD Log data
	bool lcdlog_keypress(char);
	char log_lines[LOG_LINES][LCD_X];
	int log_offset;
	void lcd_log(char *, uint8_t);

	// LCD File data
	bool lcdfilequeue_keypress(char);
	int  filequeuelen;
	int  filequeueindex;
	bool filequeuerunning;
	char filequeue[FILE_QUEUE_LEN][LCD_MAX_FILE_LEN];
	bool lcdfilequeue_addfile(char *);
	bool lcdfilequeue_delfile(int);
	void lcdfilequeue_stop();
	void lcdfilequeue_start();
	void lcdfilequeue_beginnext();

	char currentfile[LCD_MAX_FILE_LEN];
	bool lcdfile_keypress(char);
	void lcdfile_nextfile();

	// MENU
	bool lcdmenu_keypress(char);
	int menuselection;
	
	typedef enum LCD_MODE_ENUM { LCD_TEMP,LCD_ADDFILE,LCD_LOG,LCD_FILEQUEUE,LCD_MENU,LCD_MOTOR } LCD_MODE;
	LCD_MODE currentmode;


public:
	void switchLCD_TEMP();
	void switchLCD_LOG();
	void switchLCD_FILEQUEUE();
	void switchLCD_ADDFILE();
	void switchLCD_MENU();
	void switchLCD_MOTOR();
	void destroy_universe();
	void doPause();
	void abort_current_print();
	void doReset();


private:
#endif

	// Keypad support; should move into it's own class eventually.
#if HAS_KEYPAD
	uint8_t scanKeys(uint8_t);
	char	getPressedKey();
	void	initKeypad();
#endif

	static Motherboard motherboard;
public:
	/// Reset the motherboard to its initial state.
	/// This only resets the board, and does not send a reset
	/// to any attached toolheads.
	void reset();

	// SJHACK: used to update LCD
	void runMotherboardSlice();
#if HAS_LCD
	void doLcdUpdate();
	void doDataUpdate();
	void ecCommandSniffer(const OutPacket&);
	void querySniffer(const InPacket&, const OutPacket&);
#endif
#if HAS_KEYPAD
	void doKeypadUpdate();
#endif
	/// Get the UART that communicates with the host.
	UART& getHostUART() { return UART::getHostUART(); }
	/// Get the UART that communicates with the toolhead.
	UART& getSlaveUART() { return UART::getSlaveUART(); }

	/// Count the number of steppers available on this board.
	const int getStepperCount() const { return STEPPERS; }
	/// Get the stepper interface for the nth stepper.
	StepperInterface& getStepperInterface(int n)
	{
		return stepper[n];
	}

	/// Get the number of microseconds that have passed since
	/// the board was initialized.  This value will wrap after
	/// 2**32 microseconds (ca. 70 minutes); callers should compensate for this.
	micros_t getCurrentMicros();

	/// Get the power supply unit interface.
	PSU& getPSU() { return psu; }

	/// Write an error code to the debug pin.
	void indicateError(int errorCode);
	/// Get the current error being displayed.
	uint8_t getCurrentError();

	/// Get the motherboard instance.
	static Motherboard& getBoard() { return motherboard; }

	/// Perform the timer interrupt routine.
	void doInterrupt();
};

#endif // BOARDS_RRMBV12_MOTHERBOARD_HH_
