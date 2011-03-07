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

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "Motherboard.hh"
#include "Configuration.hh"
#include "Steppers.hh"
#include "Command.hh"
#if HAS_LCD
#include "LiquidCrystal.hh"
#include "Tool.hh"
#include "Commands.hh"
#include "stdio.h"
#include "string.h"
#include "SDCard.hh"
#include "Host.hh"
#include "Main.hh"
#endif
/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard;

#if HAS_LCD
LiquidCrystal lcd(LCD_RS_PIN, LCD_RW_PIN, LCD_E_PIN, LCD_0_PIN, LCD_1_PIN, LCD_2_PIN, LCD_3_PIN, LCD_4_PIN, LCD_5_PIN, LCD_6_PIN, LCD_7_PIN);
#endif

/// Create motherboard object
Motherboard::Motherboard() {
	/// Set up the stepper pins on board creation
#if STEPPER_COUNT > 0
	stepper[0] = StepperInterface(X_DIR_PIN,X_STEP_PIN,X_ENABLE_PIN,X_MAX_PIN,X_MIN_PIN, ESTOP_PIN);
#endif
#if STEPPER_COUNT > 1
	stepper[1] = StepperInterface(Y_DIR_PIN,Y_STEP_PIN,Y_ENABLE_PIN,Y_MAX_PIN,Y_MIN_PIN, ESTOP_PIN);
#endif
#if STEPPER_COUNT > 2
	stepper[2] = StepperInterface(Z_DIR_PIN,Z_STEP_PIN,Z_ENABLE_PIN,Z_MAX_PIN,Z_MIN_PIN, ESTOP_PIN);
#endif
#if STEPPER_COUNT > 3
	stepper[3] = StepperInterface(A_DIR_PIN,A_STEP_PIN,A_ENABLE_PIN,Pin(),Pin(), ESTOP_PIN);
#endif
#if STEPPER_COUNT > 4
	stepper[4] = StepperInterface(B_DIR_PIN,B_STEP_PIN,B_ENABLE_PIN,Pin(),Pin(), ESTOP_PIN);
#endif
}

/// Reset the motherboard to its initial state.
/// This only resets the board, and does not send a reset
/// to any attached toolheads.
void Motherboard::reset() {
	indicateError(0); // turn off blinker
	// Init and turn on power supply
	getPSU().init();
	getPSU().turnOn(true);
	// Init steppers
	// NB: for now, we are turning on Z hold for these boards!
	steppers::setHoldZ(true);
	for (int i = 0; i < STEPPER_COUNT; i++) {
		stepper[i].init(i);
	}
	// Initialize the host and slave UARTs
	getHostUART().enable(true);
	getHostUART().in.reset();
	getSlaveUART().enable(true);
	getSlaveUART().in.reset();
	// Reset and configure timer 1, the microsecond and stepper
	// interrupt timer.
	TCCR1A = 0x00;
	TCCR1B = 0x09;
	TCCR1C = 0x00;
	OCR1A = INTERVAL_IN_MICROSECONDS * 16;
	TIMSK1 = 0x02; // turn on OCR1A match interrupt
	// Reset and configure timer 2, the debug LED flasher timer.
	TCCR2A = 0x00;
	TCCR2B = 0x07; // prescaler at 1/1024
	TIMSK2 = 0x01; // OVF flag on
#if HAS_LCD 
	initLCD();
#endif // HAS_LCD

	// Configure the debug pin.
	DEBUG_PIN.setDirection(true);

#if HAS_KEYPAD
	initKeypad();
#endif

}


#if HAS_KEYPAD

char _kp_keymap[KP_ROWS][KP_COLS] = KP_KEYMAP;
uint8_t _kp_debounce[KP_ROWS][KP_COLS] = {0};
Pin  _kp_colpins[]  = KP_COL_PINS;
Pin  _kp_rowpins[]  = KP_ROW_PINS;
void Motherboard::initKeypad()
{
	for(int x=0;x<KP_COLS;x++)
	{
		_kp_colpins[x].setDirection(false);
		_kp_colpins[x].setValue(false);
	}
	for(int x=0;x<KP_ROWS;x++)
	{
		_kp_rowpins[x].setDirection(false);
		_kp_rowpins[x].setValue(true);
	}
}

uint8_t Motherboard::scanKeys(uint8_t colnum)
{
	_kp_colpins[colnum].setDirection(true);
	_kp_colpins[colnum].setValue(false);

	uint8_t ret = 0;
	for(int x=0; x<KP_ROWS; x++)
		if(!_kp_rowpins[x].getValue())
			ret |= 1 << x;

	_kp_colpins[colnum].setDirection(false);
	_kp_colpins[colnum].setValue(false);

	return ret;
}

// Written this way because there's no reason in theory we can't return 
// multiple keys but I'm not utilizing that possibility presently.
micros_t _kp_db_timer = 0;
char _kp_db_char = 0;
char Motherboard::getPressedKey()
{
	micros_t now = Motherboard::getBoard().getCurrentMicros();

	int colscan = 0;
	for(int x=0;x<KP_COLS; x++)
	{
		colscan = scanKeys(x);
		if(colscan)
			for(int y=0; y<KP_ROWS; y++)
				if(colscan & (1 << y))
				{
					char fc = _kp_keymap[y][x];
					if(fc == _kp_db_char && _kp_db_timer <= now)
						return fc;
					if(fc != _kp_db_char)
					{
						_kp_db_char = fc;
						_kp_db_timer = now + KP_DEBOUNCE;
					}
					return 0;
				}
	}
	if(_kp_db_timer && _kp_db_timer < now)
	{
		_kp_db_timer = 0;
		_kp_db_char = 0;
	}
	return 0;
}
#endif // HAS_KEYPAD
	


// SJHACK:
// Command slice for the mobo itself?
void Motherboard::runMotherboardSlice()
{
#if HAS_LCD
	doLcdUpdate(); 	// Draws the LCD
	doDataUpdate(); // Gets toolhead information
#endif
#if HAS_KEYPAD
	doKeypadUpdate(); // Reads pressed keys
#endif
	return;
}



#if HAS_LCD
uint8_t _lcd_linestarts[] = LCD_LINESTARTS;

void Motherboard::initLCD() {
	// INIT VARS:

	// Stored temperature data
	for(int x=0; x<NUM_TOOLS; x++)
	{
		HT[x] = 0;
		HTT[x] = 0;
		SHT[x] = 0;
	}
	PT = 0;
	PTT = 0;
	SPT = 0;

	// File handler
	memset(currentfile, 0, LCD_MAX_FILE_LEN);
	filequeuelen = 0;
	filequeueindex = 0;
	filequeuerunning=false;
	for(int x=0; x<FILE_QUEUE_LEN;x++)
		memset(filequeue[x], 0, LCD_MAX_FILE_LEN);

	// LCD-based logging
	log_offset = 0;
	for(int x=0; x<LOG_LINES; x++)
	{
		memset(log_lines[x], ' ', LCD_X);
	}

	// MENU
	menuselection=0;

	// MOTOR CONTROL
	lcd_motormovetime = 1;
	lcd_motormovemms  = 10;
	lcd_motorextrude1 = 0;
	lcd_motorextrude2 = 0;
	lcd_mecmode = false;

	// STARTUP LCD:
	lcd_log("LCD Startup.", 12);
	lcd.begin(LCD_X,LCD_Y,_lcd_linestarts);
	lcd.noAutoscroll();
	switchLCD_TEMP();
	lcd_log("LCD Initialized.", 16);
}

void Motherboard::switchLCD_TEMP()
{
	currentmode = LCD_TEMP;
	lcd.clear();
	for(int x = 0; x < NUM_TOOLS; x++)
	{

		lcd.setCursor(0,x);
		lcd.write('T');
		if(LCD_X > 16)
		{
			lcd.writeint16(x, 1);
			lcd.write(':');
		}
		lcd.writeint16(HT[x],100);
		lcd.write('/');
		lcd.writeint16(SHT[x],100);
		if(PLATFORM_HEATER_TOOLNUM == x)
		{
			if(LCD_X > 16)
				lcd.write(' ');
			lcd.write('P');
			if(LCD_X > 16)
				lcd.write(':');

			lcd.writeint16(PT,100);
			lcd.write('/');
			lcd.writeint16(SPT,100);
		}
		else if(LCD_EXTRACHARS > 0 and NUM_TOOLS >= LCD_Y)
		{
			lcd.writestr(LCD_EXTRASTRING,LCD_EXTRACHARS);
		}
	}


	if(LCD_EXTRACHARS > 0 and NUM_TOOLS < LCD_Y)
	{
#if LCD_FULLADDRESS
		lcd.setCursor(LCD_X - LCD_EXTRACHARS,LCD_Y-1);
#else
		lcd.setCursor(0,LCD_Y-1); // Only able to address every other char.
#endif
		lcd.writestr(LCD_EXTRASTRING,LCD_EXTRACHARS);
	}
}

void Motherboard::switchLCD_LOG()
{
	currentmode = LCD_LOG;
	lcd.clear();

	for(int x=LCD_Y; x > 0; x--)
	{
		lcd.setCursor(0,LCD_Y-x);
		lcd.writestr(log_lines[LOG_LINES - x - log_offset], LCD_X);	
	}
}


char *menu_lines[] = 
{ 
	"MOTOR MENU",
	"TEMPERATURE MENU",
	"FILE QUEUE",
	"MESSAGE LOG",
	"ABORT PRINT",
	"RESET PRINTER",
	"DESTROY UNIVERSE",
};
void (Motherboard::*menu_funcs[])() =
{
	&Motherboard::switchLCD_MOTOR,
	&Motherboard::switchLCD_TEMP,
	&Motherboard::switchLCD_FILEQUEUE,
	&Motherboard::switchLCD_LOG,
	&Motherboard::abort_current_print,
	&Motherboard::doReset,
	&Motherboard::destroy_universe,
};
uint8_t MENU_ENTRIES = 7;

void Motherboard::doPause()
{
	InPacket foo;
	OutPacket bar;

	// From Host.hh
	handlePause(foo, bar);
}

void Motherboard::doReset()
{
	::reset(false);
}

void Motherboard::abort_current_print()
{
	if(sdcard::isPlaying())
	{
		sdcard::finishPlayback();
		command::reset();
		lcd_log("ABORTED PRINT", 13);
	}
	else
		lcd_log("NOTHING TO ABORT", 16);
}

void Motherboard::destroy_universe()
{
	lcd_log("ERROR DESTUNIV", 14);
}

void Motherboard::switchLCD_MENU()
{
	currentmode = LCD_MENU;
	lcd.clear();
	int menulen = strlen(menu_lines[menuselection]);
	lcd.writestr(menu_lines[menuselection], LCD_X < menulen ? LCD_X : menulen);
	if(LCD_Y>1)
	{
		lcd.setCursor(0,1);
		lcd.writestr("8=NEXT 5=RUN", 12);
	}
}

void Motherboard::switchLCD_MOTOR()
{
	currentmode = LCD_MOTOR;
	lcd.clear();
	char lcdbuf[25];
	lcd.writestr("T:",2);
	lcd.writeint16(lcd_motormovetime,100);
	lcd.writestr("s E:", 4);
	sprintf(lcdbuf, "%3d", lcd_motorextrude1);
	lcd.writestr(lcdbuf,3);
	if(NUM_TOOLS > 1)
	{
		lcd.write(' ');
		sprintf(lcdbuf, "%3d", lcd_motorextrude2);
		lcd.writestr(lcdbuf,3);
	}
	lcd.setCursor(0,1);
	sprintf(lcdbuf, "S:%2dmm/s", lcd_motormovemms);
	lcd.writestr(lcdbuf,8);
	if(lcd_mecmode)
		lcd.writestr("  *EXTD*",8);
}


bool Motherboard::lcdmotor_keypress(char key)
{
	int32_t movesteps = lcd_motormovetime * lcd_motormovemms;
	int32_t movestepsa = ((lcd_motormovetime * lcd_motorextrude1) * A_STEPS_PER_MM_TIMES_1000) / 1000;
	int32_t movestepsb = ((lcd_motormovetime * lcd_motorextrude2) * B_STEPS_PER_MM_TIMES_1000) / 1000;
	int32_t movetime  = lcd_motormovetime * 1000000;
	char errbuf[16];
	
	bool handled = false;
	switch(key)
	{
		case '1':
			if(lcd_mecmode)
				lcd_motorextrude1+= lcd_motorextrude1< 10 ? 1 : 5;
			else
				lcd_motormovetime += lcd_motormovetime < 10 ? 1 : 5;
			handled = true;	
			break;
		case '7':
			if(lcd_mecmode)
				lcd_motorextrude1 -= lcd_motorextrude1< 10 ? 1 : 5;
			else
				lcd_motormovetime -= lcd_motormovetime < 10 ? 1 : 5;
			handled = true;
			break;
		case '*':
			if(lcd_mecmode)
				lcd_motorextrude2 -= lcd_motorextrude2< 10 ? 1 : 5;
			else			
				lcd_motormovemms -= lcd_motormovemms < 10 ? 1 : 5;
			handled = true;	
			break;
		case '#':
			if(lcd_mecmode)
				lcd_motorextrude2 += lcd_motorextrude2< 10 ? 1 : 5;
			else
				lcd_motormovemms += lcd_motormovemms < 10 ? 1: 5;
			handled = true;
			break;
		case '4':
			movesteps *= X_STEPS_PER_MM_TIMES_1000;
			movesteps /= 1000;
			genmovement(movesteps, 0, 0, movestepsa, movestepsb, movetime);
			handled = true;	
			break;
		case '6':
			movesteps *= X_STEPS_PER_MM_TIMES_1000;
			movesteps /= -1000;
			genmovement(movesteps, 0, 0, movestepsa, movestepsb, movetime);
			handled = true;
			break;
		case '8':
			movesteps *= Y_STEPS_PER_MM_TIMES_1000;
			movesteps /= 1000;
			genmovement(0, movesteps, 0, movestepsa, movestepsb, movetime);
			handled = true;	
			break;
		case '2':
			movesteps *= Y_STEPS_PER_MM_TIMES_1000;
			movesteps /= -1000;
			genmovement(0, movesteps, 0, movestepsa, movestepsb, movetime);
			handled = true;
			break;
		case '3':
			movesteps *= Z_STEPS_PER_MM_TIMES_1000;
			movesteps /= 1000;
			genmovement(0, 0, movesteps, movestepsa, movestepsb, movetime);
			handled = true;	
			break;
		case '9':
			movesteps *= Z_STEPS_PER_MM_TIMES_1000;
			movesteps /= -1000;
			genmovement(0, 0, movesteps, movestepsa, movestepsb, movetime);
			handled = true;
			break;
		case '5':
			lcd_mecmode = !lcd_mecmode;
			handled = true;
			break;
	}

	if(lcd_motormovetime < 1) lcd_motormovetime = 1;
	if(lcd_motormovetime > 100) lcd_motormovetime = 100;

	if(lcd_motormovemms < 1) lcd_motormovemms = 1;
	if(lcd_motormovemms > 50) lcd_motormovemms = 50;

	if(lcd_motorextrude1 < -50) lcd_motorextrude1 = -50;
	if(lcd_motorextrude1 > 50) lcd_motorextrude1 = 50;

	if(lcd_motorextrude2 < -50) lcd_motorextrude2 = -50;
	if(lcd_motorextrude2 > 50) lcd_motorextrude2 = 50;



	if(handled && (key == '1' || key == '7'|| key == '*' || key == '#' || key == '5'))
	{
		switchLCD_MOTOR(); // refresh display
		sprintf(errbuf, "E1 %10d", lcd_motorextrude1);
		lcd_log(errbuf, 13);
		sprintf(errbuf, "E2 %10d", lcd_motorextrude2);
		lcd_log(errbuf, 13);
	}


	return handled;
}

	


bool Motherboard::lcdmenu_keypress(char key)
{
	bool handled = false;
	switch(key)
	{
		case '2':
			menuselection--;
			handled = true;	
			break;
		case '8':
			menuselection++;
			handled = true;
			break;
		case '#':
		case '5':
			if(menu_funcs[menuselection] != NULL)
				(*this.*menu_funcs[menuselection])();
			handled = true;
			break;
	}

	if(menuselection >= MENU_ENTRIES) menuselection = 0;
	if(menuselection < 0) menuselection = MENU_ENTRIES -1;

	if(handled && (key == '2' || key == '8'))
		switchLCD_MENU(); // refresh display

	return handled;
}

void Motherboard::switchLCD_FILEQUEUE()
{
	currentmode = LCD_FILEQUEUE;
	lcd.clear();

	if(filequeuelen == 0)
	{
		lcd.writestr("NO QUEUE.", 9);
	}
	else
	{
		int fnlen = strlen(filequeue[filequeueindex]);
		lcd.writestr(filequeue[filequeueindex], fnlen < LCD_X ? fnlen : LCD_X);
	}

	if(LCD_Y > 1)
	{
		lcd.setCursor(0,1);
		lcd.writeint16(filequeueindex,100);
		lcd.writestr(" *=DEL #=ADD",12);
	}
}

bool Motherboard::lcdfilequeue_keypress(char key)
{
	bool handled = false;
	switch(key)
	{
		case '2':
			filequeueindex--;
			handled = true;	
			break;
		case '8':
			filequeueindex++;
			handled = true;
			break;
		case '4':
			lcdfilequeue_stop();
			handled = true;
			break;
		case '6':
			lcdfilequeue_start();
			handled = true;
			break;
		case '*':
			lcdfilequeue_delfile(filequeueindex);
			handled = true;
			break;
		case '5':
		case '#':
			switchLCD_ADDFILE();
			handled = true;
			break;
	}

	if(filequeueindex >= filequeuelen) filequeueindex = filequeuelen-1;
	if(filequeueindex < 0) filequeueindex = 0;

	if(handled && key != '#' && key != '5')
		switchLCD_FILEQUEUE(); // refresh display

	return handled;
}

void Motherboard::lcdfilequeue_beginnext()
{
	if(!filequeuelen)
		return;
	if(sdcard::isPlaying())
		return;

	sdcard::startPlayback(filequeue[0]);
	char logbuf[LCD_MAX_FILE_LEN+3];
	sprintf(logbuf,"DQ:%s", filequeue[0]);
	lcd_log(logbuf,LCD_X);
	lcdfilequeue_delfile(0);
	if(!filequeuelen) filequeuerunning = false;

	if(currentmode == LCD_FILEQUEUE)
		switchLCD_FILEQUEUE(); // Refresh
}

void Motherboard::lcdfilequeue_stop()
{
	filequeuerunning = false;
	lcd_log("QUEUE STOP",10);
}

void Motherboard::lcdfilequeue_start()
{
	filequeuerunning = true;
	lcd_log("QUEUE START",11);
}


bool Motherboard::lcdfilequeue_addfile(char *str)
{
	if(filequeuelen >= FILE_QUEUE_LEN - 1)
		return false;

	strncpy(filequeue[filequeuelen], str, LCD_MAX_FILE_LEN);
	filequeuelen++;

	return true;
}

bool Motherboard::lcdfilequeue_delfile(int index)
{
	if(filequeuelen == 0 || index < 0 || index >= filequeuelen)
		return false;

	filequeuelen--;
	
	for(int x=index;x < FILE_QUEUE_LEN-1; x++)
		strncpy(filequeue[x], filequeue[x+1], LCD_MAX_FILE_LEN);

	if(filequeueindex >= filequeuelen)
		filequeueindex = filequeuelen - 1;
	if(filequeueindex < 0)
		filequeueindex = 0;

	return true;
}




void Motherboard::switchLCD_ADDFILE()
{
	currentmode = LCD_ADDFILE;
	lcd.clear();

	if(currentfile[0] == 0)
		lcdfile_nextfile();

	if(currentfile[0] == 0)
	{
		lcd.writestr("NO FILES.", 9);
		return;
	}

	int fnlen = strlen(currentfile) - 1;
	lcd.writestr(currentfile, fnlen < LCD_X ? fnlen : LCD_X);
	if(LCD_Y > 1)
	{
		lcd.setCursor(0,1);
		lcd.writestr("PRESS 5 TO QUEUE",16);
	}
}

bool Motherboard::lcdfile_keypress(char key)
{
	bool handled = false;
	switch(key)
	{
		case '2':
			// SJTODO: scroll backwards?
		case '8':
			lcdfile_nextfile();
			handled = true;
			break;
		case '5':
		case '#':
			lcdfilequeue_addfile(currentfile);
			switchLCD_FILEQUEUE();
			handled = true;
			break;
		case '*':
			switchLCD_FILEQUEUE();
			handled = true;
			break;
	}

	if(handled && (key == '2' || key == '8'))
		switchLCD_ADDFILE(); // refresh display

	return handled;

}

void Motherboard::lcdfile_nextfile()
{
	sdcard::SdErrorCode e;
	char err[LCD_X];
	memset(err, ' ', LCD_X);
	bool start = false;

	if(currentfile[0] == 0)
	{
		start = true;
		e = sdcard::directoryReset();
		if(e != sdcard::SD_SUCCESS)
		{
			sprintf(err,"SDRESET FAIL %03d", e);
			lcd_log(err, 16);
		}
	}


        // Ignore dot-files
        do {
                e = sdcard::directoryNextEntry(currentfile,LCD_MAX_FILE_LEN);
                if (currentfile[0] == '\0') break;
        } while (e == sdcard::SD_SUCCESS && currentfile[0] == '.');
	if(e != sdcard::SD_SUCCESS)
	{
		sprintf(err,"SDNEXT FAIL %03d", e);
		lcd_log(err, 15);
	}
	if(currentfile[0] == 0 and start)
	{
		sprintf(err,"SD NO FILE", e);
		lcd_log(err, 10);
	}
}

void Motherboard::lcd_log(char *str, uint8_t len)
{
	for(int x=0; x < LOG_LINES-1; x++)
	{
		strncpy(log_lines[x],log_lines[x+1],LCD_X);
	}

	for(int x=0; x < LCD_X; x++)
	{
		if(x>=len)
			log_lines[LOG_LINES-1][x] = ' ';
		else
			log_lines[LOG_LINES-1][x] = str[x];
	}

	if(currentmode == LCD_LOG)
		switchLCD_LOG(); // refresh display
}

bool Motherboard::lcdlog_keypress(char key)
{
	bool handled = false;
	switch(key)
	{
		case '2':
			log_offset++;
			handled = true;	
			break;
		case '8':
			log_offset--;
			handled = true;
			break;
	}

	if(log_offset < 0) log_offset = 0;
	if(log_offset > LOG_LINES - LCD_Y) log_offset = LOG_LINES - LCD_Y;

	if(handled)
		switchLCD_LOG(); // refresh display

	return handled;
}

#endif // HAS_LCD



/// Get the number of microseconds that have passed since
/// the board was booted.
micros_t Motherboard::getCurrentMicros() {
	micros_t micros_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		micros_snapshot = micros;
	}
	return micros_snapshot;
}

/// Run the motherboard interrupt
void Motherboard::doInterrupt() {
	micros += INTERVAL_IN_MICROSECONDS;
	// Do not move steppers if the board is in a paused state
	//if (command::isPaused()) return;
	steppers::doInterrupt();
}

/// Timer one comparator match interrupt
ISR(TIMER1_COMPA_vect) {
	Motherboard::getBoard().doInterrupt();
}

/// Number of times to blink the debug LED on each cycle
volatile uint8_t blink_count = 0;

/// The current state of the debug LED
enum {
	BLINK_NONE,
	BLINK_ON,
	BLINK_OFF,
	BLINK_PAUSE
} blink_state = BLINK_NONE;

/// Write an error code to the debug pin.
void Motherboard::indicateError(int error_code) {
#if HAS_LCD
	if(error_code != 0)
	{
		char buf[LCD_X];
		memset(buf, ' ', LCD_X);
		sprintf(buf, "MOBO ERR: %03d", error_code);
		lcd_log(buf, 13);
	}
#endif
	if (error_code == 0) {
		blink_state = BLINK_NONE;
		DEBUG_PIN.setValue(false);
	}
	else if (blink_count != error_code) {
		blink_state = BLINK_OFF;
	}
	blink_count = error_code;
}

/// Get the current error code.
uint8_t Motherboard::getCurrentError() {
	return blink_count;
}

/// Timer2 overflow cycles that the LED remains on while blinking
#define OVFS_ON 18
/// Timer2 overflow cycles that the LED remains off while blinking
#define OVFS_OFF 18
/// Timer2 overflow cycles between flash cycles
#define OVFS_PAUSE 80

/// Number of overflows remaining on the current blink cycle
int blink_ovfs_remaining = 0;
/// Number of blinks performed in the current cycle
int blinked_so_far = 0;
#if HAS_LCD
/// Count down to Temperature/LCD update
int data_update_interrupts = 0;
bool data_update = false;
bool lcd_char_update = false;
micros_t lcd_micro_count = 0;
#endif

/// Timer 2 overflow interrupt
ISR(TIMER2_OVF_vect) {
#if HAS_LCD
// SJHACK: If my math is right, this happens about 250 times a second.  We'll piggyback on that to indicate it's time to
// update the LCD.
	if(data_update_interrupts++ > 250/NUM_TOOLS) {
		data_update_interrupts = 0;
		data_update = true;
	}
        
	micros_t now = Motherboard::getBoard().getCurrentMicros();
	if(now < lcd_micro_count || now - lcd_micro_count > 100)
	{
		lcd_char_update = true;
	}
	lcd_micro_count = now;

#endif
	
	if (blink_ovfs_remaining > 0) {
		blink_ovfs_remaining--;
	} else {
		if (blink_state == BLINK_ON) {
			blinked_so_far++;
			blink_state = BLINK_OFF;
			blink_ovfs_remaining = OVFS_OFF;
			DEBUG_PIN.setValue(false);
		} else if (blink_state == BLINK_OFF) {
			if (blinked_so_far >= blink_count) {
				blink_state = BLINK_PAUSE;
				blink_ovfs_remaining = OVFS_PAUSE;
			} else {
				blink_state = BLINK_ON;
				blink_ovfs_remaining = OVFS_ON;
				DEBUG_PIN.setValue(true);
			}
		} else if (blink_state == BLINK_PAUSE) {
			blinked_so_far = 0;
			blink_state = BLINK_ON;
			blink_ovfs_remaining = OVFS_ON;
			DEBUG_PIN.setValue(true);
		}
	}
}

#if HAS_LCD

void Motherboard::ecCommandSniffer(const OutPacket& to_ec)
{
	uint8_t toolnum = to_ec.read8(0);
	uint8_t toolcmd = to_ec.read8(1);
	
	switch (toolcmd) {
		case SLAVE_CMD_SET_TEMP:
			SHT[toolnum] = to_ec.read16(2);
			return;
		case SLAVE_CMD_SET_PLATFORM_TEMP:
			if(toolnum == PLATFORM_HEATER_TOOLNUM)
				SPT = to_ec.read16(2);
			return;
	}
}

void Motherboard::querySniffer(const InPacket& from_host, const OutPacket& to_host)
{
	uint8_t command = from_host.read8(0);
	switch (command) {
	case HOST_CMD_TOOL_QUERY:
		uint8_t toolnum = from_host.read8(1);
		uint8_t toolcmd = from_host.read8(2);
		switch (toolcmd) {
			case SLAVE_CMD_GET_TEMP:
				HT[toolnum]   = to_host.read16(1);
				HTT[toolnum]  = Motherboard::getBoard().getCurrentMicros();
				return;
			case SLAVE_CMD_GET_PLATFORM_TEMP:
				if(toolnum == PLATFORM_HEATER_TOOLNUM) {
					PT = to_host.read16(1);
					PTT = Motherboard::getBoard().getCurrentMicros();
				}
				return;
		}
		
	}
}

void Motherboard::doLcdUpdate()
{
	if(lcd_char_update == true)
	{
		lcd_char_update = false;
		lcd.handleUpdates();
	}

	if(filequeuerunning && !sdcard::isPlaying())
		lcdfilequeue_beginnext();
		

}

void Motherboard::doDataUpdate()
{
	static int dataupdatenum = 0;
	if(data_update == true)
	{
		if(++dataupdatenum > NUM_TOOLS)
			dataupdatenum = 0;
		data_update = false;
		micros_t now = Motherboard::getBoard().getCurrentMicros();
		if(dataupdatenum != NUM_TOOLS)
		{
			if(now < HTT[dataupdatenum] || now - HTT[dataupdatenum] > LCD_TEMP_UPDATE_INTERVAL_MICROS)
			{
				HT[dataupdatenum] = grabToolInfo(dataupdatenum,SLAVE_CMD_GET_TEMP);
				HTT[dataupdatenum] = now;
			}

			if(currentmode == LCD_TEMP)
			{
				lcd.setCursor(1,dataupdatenum);
				if(LCD_X > 16) 
					lcd.write(':');
				lcd.writeint16(HT[dataupdatenum],100);
				lcd.write('/');
				lcd.writeint16(SHT[dataupdatenum],100);
			}
		}
		else
		{
			if(now < PTT || now - PTT > LCD_TEMP_UPDATE_INTERVAL_MICROS)
			{
				PT = grabToolInfo(PLATFORM_HEATER_TOOLNUM,SLAVE_CMD_GET_PLATFORM_TEMP);
				PTT = now;
			}

			if(currentmode == LCD_TEMP)
			{

				if(LCD_X > 16)
				{
					lcd.setCursor(LCD_FULLADDRESS ? 10 : 5,NUM_TOOLS - 1);
					lcd.writestr(" P:",3);
				}
				else
				{
					lcd.setCursor(LCD_FULLADDRESS ? 8 : 4,NUM_TOOLS - 1);
					lcd.write('P');
				}

				lcd.writeint16(PT, 100);
				lcd.write('/');
				lcd.writeint16(SPT,100);
			}
		}
	}
}

uint16_t Motherboard::grabToolInfo(int toolnum, uint8_t command)
{
	uint16_t r = 999;

	if (tool::getLock()) {
		OutPacket& out = tool::getOutPacket();
                InPacket& in = tool::getInPacket();
                out.reset();
                out.append8(toolnum);
                out.append8(command);
                tool::startTransaction();
                // WHILE: bounded by timeout in runToolSlice
                while (!tool::isTransactionDone()) {
                  tool::runToolSlice();
                }
                if (!in.hasError()) {
                   r = in.read16(1);
                }
                tool::releaseLock();

	}
	return r;
}

void Motherboard::insertoverridecommand(uint8_t command, uint8_t *params, uint8_t paramslen)
{
	command::push_override(command);
	for(int x=0; x<paramslen; x++)
		command::push_override(params[x]);
}

void Motherboard::genmovement(int32_t x, int32_t y, int32_t z, int32_t a, int32_t b, int32_t us)
{
	union {
		int32_t a;
		struct { uint8_t data[4]; } b;
	} s;

	command::push_override(HOST_CMD_QUEUE_POINT_NEW);

	s.a=x;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	s.a=y;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	s.a=z;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	s.a=a;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	s.a=b;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	s.a=us;
	for(int x=0;x<4;x++) command::push_override(s.b.data[x]);
	command::push_override(0xff); // ALL AXES RELATIVE
}




#endif

#ifdef HAS_KEYPAD
void Motherboard::doKeypadUpdate()
{
	static char lastpressed = 0;
	char keypressed = getPressedKey();
	if(keypressed && keypressed != lastpressed)
	{
		//lcd_log("Keypress.", 9);

		bool handled = false;
		switch(currentmode) 
		{
			case LCD_TEMP:
			default:
				break;
			case LCD_LOG:
				handled = lcdlog_keypress(keypressed);
				break;
			case LCD_FILEQUEUE:
				handled = lcdfilequeue_keypress(keypressed);
				break;
			case LCD_ADDFILE:
				handled = lcdfile_keypress(keypressed);
				break;
			case LCD_MENU:
				handled = lcdmenu_keypress(keypressed);
				break;
			case LCD_MOTOR:
				handled = lcdmotor_keypress(keypressed);
				break;
		}

		if(!handled)
		switch(keypressed)
		{
			case '1':
				switchLCD_TEMP();
				break;
			case '3':
				switchLCD_LOG();
				break;
			case '7':
				switchLCD_FILEQUEUE();
				break;
			case '0':
				switchLCD_MENU();
				break;
			case '9':
				switchLCD_MOTOR();
			case '5':
				doPause();
				break;
		}
	}
	lastpressed = keypressed;
}

#endif
