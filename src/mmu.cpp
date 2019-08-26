//mmu.cpp

#include "planner.h"
#include "language.h"
#include "lcd.h"
#include "uart2.h"
#include "temperature.h"
#include "Configuration_prusa.h"
#include "fsensor.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "sound.h"
#include "printers.h"
#include <avr/pgmspace.h>
#include "io_atmega2560.h"
#include "AutoDeplete.h"

#ifdef TMC2130
#include "tmc2130.h"
#endif //TMC2130

#define CHECK_FINDA ((IS_SD_PRINTING || is_usb_printing) && (mcode_in_progress != 600) && !saved_printing && e_active())

#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 300000ul //5min timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000ul //timeout for P0 command: 3seconds

#ifdef MMU_HWRESET
#define MMU_RST_PIN 76
#endif //MMU_HWRESET

#ifdef MMU_DEBUG
static const auto DEBUG_PUTCHAR = putchar;
static const auto DEBUG_PUTS_P = puts_P;
static const auto DEBUG_PRINTF_P = printf_P;
#else //MMU_DEBUG
#define DEBUG_PUTCHAR(c)
#define DEBUG_PUTS_P(str)
#define DEBUG_PRINTF_P(__fmt, ...)
#endif //MMU_DEBUG

namespace
{ // MMU2S States
enum class S : uint_least8_t
{
  FindaInit,
  GetActExt,
  GetBN,
  GetVer,
  Init,
  Disabled,
  Idle,
  GetFinda,
  Wait,
  Setup
};
}

bool mmu_enabled = false;
bool mmu_ready = false;
bool mmu_fil_loaded = false; //if true: blocks execution of duplicit T-codes
bool isMMUPrintPaused = false;
bool mmu_loading_flag = false;
int lastLoadedFilament = -10;
uint16_t mmu_power_failures = 0;

uint16_t toolChanges = 0;
uint8_t mmuE0BackupCurrents[2] = {0, 0};
void shutdownE0(bool shutdown = true);
#define TXTimeout 60 //60ms
//uint8_t unload_filament_type[3] = {0, 0, 0};
uint8_t mmu_filament_types[5] = {0, 0, 0, 0, 0};
void mmu_unload_synced(uint16_t _filament_type_speed);

//idler ir sensor
static bool mmu_idl_sens = 0;
bool ir_sensor_detected = false;

//if IR_SENSOR defined, always returns true
//otherwise check for ir sensor and returns true if idler IR sensor was detected, otherwise returns false
bool check_for_ir_sensor()
{
  return true;
}

static S mmu_state = S::Disabled; //int8_t mmu_state = 0;
MmuCmd mmu_cmd = MmuCmd::None;
MmuCmd mmu_last_cmd = MmuCmd::None;
uint8_t mmu_extruder = 0;

//! This variable probably has no meaning and is planed to be removed
uint8_t tmp_extruder = 0;
int8_t mmu_finda = -1;
int16_t mmu_version = -1;
int16_t mmu_buildnr = -1;
uint32_t mmu_last_request = 0;
uint32_t mmu_last_response = 0;

//initialize mmu2 unit - first part - should be done at begining of startup process
void mmu_init(void)
{
#ifdef MMU_HWRESET
  digitalWrite(MMU_RST_PIN, HIGH);
  pinMode(MMU_RST_PIN, OUTPUT); //setup reset pin
#endif                          //MMU_HWRESET
  uart2_init();                 //init uart2
  _delay_ms(10);                //wait 10ms for sure
  mmu_reset();                  //reset mmu (HW or SW), do not wait for response
  mmu_state = S::Init;
  PIN_INP(IR_SENSOR_PIN); //input mode
  PIN_SET(IR_SENSOR_PIN); //pullup
}

//! @brief Is nozzle hot enough to move extruder wheels and do we have idler sensor?
//!
//! Do load steps only if temperature is higher then min. temp for safe extrusion and
//! idler sensor present.
//! Otherwise "cold extrusion prevented" would be send to serial line periodically
//! and watchdog reset will be triggered by lack of keep_alive processing.
//!
//! @retval true temperature is high enough to move extruder
//! @retval false temperature is not high enough to move extruder, turned
//!         off E-stepper to prevent over-heating and allow filament pull-out if necessary
bool can_extrude()
{
    if ((degHotend(active_extruder) < EXTRUDE_MINTEMP) || !ir_sensor_detected)
    {
        disable_e0();
        delay_keep_alive(100);
        return false;
    }
    return true;
}

void mmu_loop(void)
{
  uint8_t filament = 0;
  #ifdef MMU_DEBUG
  printf_P(PSTR("MMU loop, state=%d\n"), (int)mmu_state);
  #endif //MMU_DEBUG

  // Copy volitale vars as local
  unsigned char tData1 = rxData1;
  unsigned char tData2 = rxData2;
  unsigned char tData3 = rxData3;
  unsigned char tData4 = rxData4;
  unsigned char tData5 = rxData5;
  bool confPayload = confirmedPayload;

  if (txACKNext) uart2_txACK();
  if (txNAKNext) uart2_txACK(false);
  if (txRESEND) uart2_txPayload(lastTxPayload, true);
  if (confPayload) mmu_last_response = _millis();
  else {
      tData1 = ' ';
      tData2 = ' ';
      tData3 = ' ';
      tData4 = ' ';
      tData5 = ' ';
  }

  // All Notification & Status Updates between MK3S/MMU2S
  if (mmu_state > S::Disabled)
  {
    // Undates Active Extruder when every MMU is changed. eg. manual ex selection fro feed_filament
    if ((tData1 == 'A') && (tData2 == 'E') && (tData3 < 5))
    {
      printf_P(PSTR("MMU => MK3 'OK Active Extruder:%d'\n"), tmp_extruder + 1);
      tmp_extruder = tData3;
      mmu_extruder = tmp_extruder;
    }
    else if ((tData1 == 'S') && (tData2 == 'E') && (tData3 == 'T') && tData4 == 'U' && tData5 == 'P')
      mmu_state = S::Setup;
    else if ((tData1 == 'Z') && (tData2 == 'Z') && (tData3 == 'Z'))
    { // Clear MK3 Messages
      //********************
      lcd_setstatuspgm(_T(WELCOME_MSG));
      lcd_return_to_status();
      lcd_update_enable(true);
    }
    else if ((tData1 == 'Z') && (tData2 == 'Z') && (tData3 == 'R'))
    { // Advise MMU2S to reconnect
      lcd_setstatuspgm(_T(WELCOME_MSG));
      lcd_return_to_status();
      lcd_update_enable(true);
      tmp_extruder = MMU_FILAMENT_UNKNOWN;
      mmu_extruder = MMU_FILAMENT_UNKNOWN;
      mmu_enabled = false;
      mmu_reset();
      mmu_state = S::Init;
    }
    else if ((tData1 == 'Z') && (tData2 == 'L') && (tData3 == '1'))
    {                                        // MMU Loading Failed
                                              //********************
      lcd_setstatus("MMU Load Failed @MMU"); // 20 Chars
      #ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedLoad1\n"));
      #endif                                 // OCTO_NOTIFICATIONS_ON
    }
    else if ((tData1 == 'Z') && (tData2 == 'L') && (tData3 == '2'))
    {                                        // MMU Loading Failed
                                              //********************
      lcd_setstatus("MMU Load Failed @MK3"); // 20 Chars
      #ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedLoad2\n"));
      #endif // OCTO_NOTIFICATIONS_ON
      mmu_loading_flag = false;
      mmu_state = S::Wait;
    }
    else if ((tData1 == 'Z') && (tData2 == 'U'))
    {                                        // MMU Unloading Failed
                                              //********************
      lcd_setstatus(" MMU Unload Failed  "); // 20 Chars
      #ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedUnload\n"));
      #endif                                 // OCTO_NOTIFICATIONS_ON
    }
    else if ((tData1 == 'Z') && (tData2 == '1'))
    {                                        // MMU Filament Loaded
                                              //********************
      lcd_setstatus("ERR: Filament Loaded"); // 20 Chars
    }
    else if ((tData1 == 'X') && (tData2 == '1'))
    {                                        // MMU Setup Menu
                                              //********************
      lcd_setstatus("   MMU Setup Menu   "); // 20 Chars
    }
    else if ((tData1 == 'X') && (tData2 == '2'))
    {                                        // MMU Adj Bowden Len
                                              //********************
      lcd_setstatus("MMU Adj BowLen/Bond "); // 20 Chars
    }
    else if ((tData1 == 'X') && (tData2 == '5'))
    {                                        // MMU Setup Menu: Unlock EEPROM
                                              //********************
      lcd_setstatus(" MMU Unlock EEPROM  "); // 20 Chars
    }
    else if ((tData1 == 'X') && (tData2 == '6'))
    {                                        // MMU Setup Menu: Clr Unlocked EEPROM
                                              //********************
      lcd_setstatus("Clr EEPROM(unlocked)"); // 20 Chars
    }
    else if ((tData1 == 'X') && (tData2 == '7'))
    {                                   // MMU Setup Menu: Exit
                                        //********************
      lcd_setstatus("Exit Setup Menu"); // 20 Chars
      lcd_return_to_status();
      lcd_update_enable(true);
    }
    else if (tData1 == 'B')
    { // MMU Adj Fsensor to Bondtech Length
      //********************
      lcd_update_enable(false);
      lcd_clear(); //********************
      lcd_set_cursor(0, 0);
      lcd_puts_P(_i("L:  +1mm (Extrude)  "));
      lcd_set_cursor(0, 1);
      lcd_puts_P(_i("M:Unload Save & Exit"));
      lcd_set_cursor(0, 2);
      lcd_puts_P(_i("R:  -1mm (Retract)  "));
      lcd_set_cursor(0, 3);
      lcd_puts_P(_i("Current mm: "));
      lcd_set_cursor(12, 3);
      lcd_print((tData2 << 8) | (tData3));
    }
    else if (tData1 == 'V')
    { // MMU Adj Bowden Len: Loaded Message
      //********************
      lcd_update_enable(false);
      lcd_clear(); //********************
      lcd_set_cursor(0, 0);
      lcd_puts_P(_i("L: +2mm (Extrude)   "));
      lcd_set_cursor(0, 1);
      lcd_puts_P(_i("M:Unload(Save/Retry)"));
      lcd_set_cursor(0, 2);
      lcd_puts_P(_i("R: -2mm (Retract)   "));
      lcd_set_cursor(0, 3);
      lcd_puts_P(_i("Current mm: "));
      lcd_set_cursor(12, 3);
      lcd_print((tData2 << 8) | (tData3));
    }
    else if (tData1 == 'W')
    { // MMU Adj Bowden Len: Unloaded Message
      //********************
      lcd_update_enable(false);
      lcd_clear();                       //********************
      lcd_set_cursor(0, 0); lcd_puts_P(_i("L:   Save & Exit    "));
      lcd_set_cursor(0, 1);
      lcd_puts_P(_i("M:BowLen Load2Check "));
      lcd_set_cursor(0, 2);
      lcd_puts_P(_i("                    ")); //R:Set BondTech Steps"));
      lcd_set_cursor(0, 3);
      lcd_puts_P(_i("Current mm: "));
      lcd_set_cursor(12, 3);
      lcd_print((tData2 << 8) | (tData3));
    }
    else if (tData1 == 'T')
    { // MMU Report ToolChange Count
      toolChanges = ((tData2 << 8) | (tData3));
      printf_P(PSTR("MMU => MK3 '@toolChange:%d'\n"), toolChanges);
    }
  } // End of mmu_state > S::Disabled


	switch (mmu_state)
	{
	case S::Disabled:
		return;
	case S::Init:
    if ((tData1 == 'S') && (tData2 == 'T') && (tData3 == 'R'))
    {
      #ifdef MMU_DEBUG
      puts_P(PSTR("MMU => MK3 'start'"));
      puts_P(PSTR("MK3 => MMU 'S1'"));
      #endif                                 //MMU_DEBUG
      uart2_txPayload((unsigned char *)"S1---");
      mmu_state = S::GetVer;
    }
    else if (mmu_last_response + MMU_CMD_TIMEOUT < _millis())
    { //30sec after reset disable mmu
      puts_P(PSTR("MMU not responding - DISABLED"));
      mmu_state = S::Disabled;
    } // End of if STR
    return; // Exit method.
  case S::GetVer:
    if ((tData1 == 'O') && (tData2 == 'K')) {
      mmu_version = ((tData3 << 8) | (tData4));
      printf_P(PSTR("MMU => MK3 '%d'\n"), mmu_version);
      #ifdef MMU_DEBUG
      puts_P(PSTR("MK3 => MMU 'S2'"));
      #endif //MMU_DEBUG
      uart2_txPayload((unsigned char *)"S2---");
      mmu_state = S::GetBN;
    }
    return; // Exit method.
  case S::GetBN:
    if ((tData1 == 'O') && (tData2 == 'K')) {
      mmu_buildnr = ((tData3 << 8) | (tData4));
      printf_P(PSTR("MMU => MK3 '%d'\n"), mmu_buildnr);
      bool version_valid = mmu_check_version();
      if (!version_valid)
        mmu_show_warning();
      else
        puts_P(PSTR("MMU version valid"));
      uart2_txPayload((unsigned char *)"S3---");
      mmu_state = S::GetActExt;
    }
    return; // Exit method.
  case S::GetActExt:
    if ((tData1 == 'O') && (tData2 == 'K')) {
      tmp_extruder = tData3;
      mmu_extruder = tmp_extruder;
      printf_P(PSTR("MMU => MK3 'Active Extruder:%d'\n"), tmp_extruder + 1);
      #ifdef MMU_DEBUG
      puts_P(PSTR("MK3 => MMU 'P0'"));
      #endif //MMU_DEBUG
      uart2_txPayload((unsigned char *)"P0---");
      mmu_state = S::FindaInit;
    }
    return; // Exit method.
  case S::FindaInit:
    if ((tData1 == 'P') && (tData2 == 'K'))
    {
      mmu_finda = tData3;
      #ifdef MMU_DEBUG
      printf_P(PSTR("MMU => MK3 'PK%d'\n"), mmu_finda);
      #endif //MMU_DEBUG
      puts_P(PSTR("MMU - ENABLED"));
      mmu_enabled = true;
      mmu_state = S::Idle;
    }
    return; // Exit method.
  case S::Idle:
    if (mmu_cmd != MmuCmd::None)
    {
      if ((mmu_cmd >= MmuCmd::T0) && (mmu_cmd <= MmuCmd::T4))
      {
        filament = mmu_cmd - MmuCmd::T0;
        printf_P(PSTR("MK3 => MMU 'T%d'\n"), filament);
        unsigned char tempTxCMD[5] = {'T', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempTxCMD);
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::L0) && (mmu_cmd <= MmuCmd::L4))
      {
        filament = mmu_cmd - MmuCmd::L0;
        printf_P(PSTR("MK3 => MMU 'L%d'\n"), filament);
        unsigned char tempLxCMD[5] = {'L', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempLxCMD);
        mmu_state = S::Wait;
      }
      else if (mmu_cmd == MmuCmd::C0)
      {
        printf_P(PSTR("MK3 => MMU 'C0'\n"));
        uart2_txPayload((unsigned char *)"C0---");
        mmu_state = S::Wait;
        mmu_idl_sens = true;
      }
      else if (mmu_cmd == MmuCmd::U0)
      {
        if (PIN_GET(IR_SENSOR_PIN) == 0) //filament is still detected by idler sensor, printer helps with unlading 
        {
          if (can_extrude())
          {
            printf_P(PSTR("Unload 1\n"));
            current_position[E_AXIS] = current_position[E_AXIS] - MMU_LOAD_FEEDRATE * MMU_LOAD_TIME_MS*0.001;
            plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
            st_synchronize();
          }
        }
        printf_P(PSTR("MK3 => MMU 'U0'\n"));
        uart2_txPayload((unsigned char *)"U0---");
        //toolChanges = 0;
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::E0) && (mmu_cmd <= MmuCmd::E4))
      {
        filament = mmu_cmd - MmuCmd::E0;
        printf_P(PSTR("MK3 => MMU 'E%d'\n"), filament);
        unsigned char tempExCMD[5] = {'E', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempExCMD);
        mmu_state = S::Wait;
      }
      else if (mmu_cmd == MmuCmd::R0)
      {
        printf_P(PSTR("MK3 => MMU 'R0'\n"));
        uart2_txPayload((unsigned char *)"R0---");
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::F0) && (mmu_cmd <= MmuCmd::F4))
      {
        uint8_t extruder = mmu_cmd - MmuCmd::F0;
        unsigned char tempTxCMD[5] = {'F', extruder, mmu_filament_types[extruder], BLK, BLK};
        printf_P(PSTR("MK3 => MMU 'F%d %d'\n"), extruder, mmu_filament_types[extruder]);
        uart2_txPayload(tempTxCMD);
        mmu_state = S::Wait;
      }
      mmu_cmd = MmuCmd::None;
    }
    else if (((mmu_last_response + 300) < _millis()) && !mmu_loading_flag)
    {
      uart2_txPayload((unsigned char *)"P0---");
      mmu_state = S::GetFinda;
    } 
    return; // Exit method.
  case S::GetFinda:
    if (mmu_idl_sens)
    {
      if (PIN_GET(IR_SENSOR_PIN) == 0 && mmu_loading_flag)
      {
        uart2_txPayload((unsigned char *)"IRSEN");
        printf_P(PSTR("MK3 => MMU 'Filament seen at extruder'\n"));
        mmu_idl_sens = 0;
      }
    }
    if ((tData1 == 'I') && (tData2 == 'R') && (tData3 == 'S') && (tData4 == 'E') && (tData5 == 'N'))
    {
      printf_P(PSTR("MMU => MK3 'waiting for filament @ MK3 IR Sensor'\n"));
      mmu_load_step(false);
      mmu_fil_loaded = true;
      mmu_idl_sens = true;
    }
    if ((tData1 == 'P') && (tData2 == 'K'))
    {
      mmu_finda = tData3;
      #ifdef MMU_DEBUG
      printf_P(PSTR("MMU => MK3 'PK%d'\n"), mmu_finda);
      #endif //MMU_DEBUG
      if (!mmu_finda && CHECK_FSENSOR && fsensor_enabled)
      {
        #ifdef OCTO_NOTIFICATIONS_ON
        printf_P(PSTR("// action:m600\n"));
        #endif // OCTO_NOTIFICATIONS_ON
        fsensor_stop_and_save_print();
        enquecommand_front_P(PSTR("PRUSA fsensor_recover")); //then recover
        ad_markDepleted(mmu_extruder);
        if (lcd_autoDepleteEnabled() && !ad_allDepleted())
        {
            enquecommand_front_P(PSTR("M600 AUTO")); //save print and run M600 command
        }
        else
        {
            enquecommand_front_P(PSTR("M600")); //save print and run M600 command
        }
      }
      mmu_state = S::Idle;
      if (mmu_cmd == MmuCmd::None)
				mmu_ready = true;
    }
    else if (((mmu_last_request + MMU_P0_TIMEOUT) < _millis()))
      mmu_state = S::Idle;  // RMM was Wait
    return; // Exit method.
  case S::Wait:
    if (mmu_idl_sens)
    {
      if (PIN_GET(IR_SENSOR_PIN) == 0 && mmu_loading_flag)
      {
        uart2_txPayload((unsigned char *)"IRSEN");
        printf_P(PSTR("MK3 => MMU 'Filament seen at extruder'\n"));
        mmu_idl_sens = 0;
      }
    }
    if ((tData1 == 'I') && (tData2 == 'R') && (tData3 == 'S') && (tData4 == 'E') && (tData5 == 'N'))
    {
      printf_P(PSTR("MMU => MK3 'waiting for filament @ MK3 IR Sensor'\n"));
      mmu_load_step(false);
      mmu_fil_loaded = true;
      mmu_idl_sens = true;
    }
    if (tData1 == 'U')
    {
      mmu_unload_synced((tData2 << 8) | (tData3));
      printf_P(PSTR("MMU => MK3 Unload Feedrate: %d%d\n"), tData2, tData3);
    }
    else if ((tData1 == 'O') && (tData2 == 'K') && (tData3 == '-'))
    {
      printf_P(PSTR("MMU => MK3 'ok'\n"));
      mmu_ready = true;
      mmu_state = S::Idle;
    }
    return; // Exit method.
  }
} // End of mmu_loop() method.

void mmu_reset(void)
{
    #ifdef MMU_HWRESET //HW - pulse reset pin
  digitalWrite(MMU_RST_PIN, LOW);
  _delay_us(50);
  printf_P(PSTR("MK3S => MMU2S : HWR RESET\n"));
  digitalWrite(MMU_RST_PIN, HIGH);
  #else //SW - send X0 command
  uart2_txPayload((unsigned char *)"X0---");
  #endif
} // End of mmu_reset() method.

void mmu_set_filament_type(uint8_t extruder, uint8_t filament)
{
  printf_P(PSTR("MK3 => 'F%d %d'\n"), extruder, filament);
  unsigned char tempFx[5] = {'F', extruder, filament, BLK, BLK};
  uart2_txPayload(tempFx);
} // End of mmu_set_filament_type() method.

//! @brief Enqueue MMUv2 command
//!
//! Call manage_response() after enqueuing to process command.
//! If T command is enqueued, it disables current for extruder motor if TMC2130 driver present.
//! If T or L command is enqueued, it marks filament loaded in AutoDeplete module.
void mmu_command(MmuCmd cmd)
{
  if (((cmd >= MmuCmd::T0) && (cmd <= MmuCmd::T4)) || ((cmd >= MmuCmd::E0) && (cmd <= MmuCmd::E4)))
  {
    disable_e0();
  }
  mmu_last_cmd = mmu_cmd;
  mmu_cmd = cmd;
  mmu_ready = false;
}

void mmu_unload_synced(uint16_t _filament_type_speed)
{
  st_synchronize();
  shutdownE0(false);
  current_position[E_AXIS] -= 70;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], _filament_type_speed, active_extruder);
  st_synchronize();
  disable_e0();
}

static void get_response_print_info(uint8_t move)
{
  printf_P(PSTR("mmu_get_response - begin move: "), move);
  switch (move)
  {
  case MMU_LOAD_MOVE:
    printf_P(PSTR("load\n"));
    break;
  case MMU_UNLOAD_MOVE:
    printf_P(PSTR("unload\n"));
    break;
  case MMU_TCODE_MOVE:
    printf_P(PSTR("T-code\n"));
    break;
  case MMU_NO_MOVE:
    printf_P(PSTR("no move\n"));
    break;
  default:
    printf_P(PSTR("error: unknown move\n"));
    break;
  }
}

bool mmu_get_response(uint8_t move)
{

	get_response_print_info(move);
	KEEPALIVE_STATE(IN_PROCESS);
	while (mmu_cmd != MmuCmd::None)
	{
		delay_keep_alive(100);
	}

	while (!mmu_ready)
	{
		if ((mmu_state != S::Wait) && (mmu_last_cmd == MmuCmd::None))
			break;

		switch (move) {
			case MMU_LOAD_MOVE:
			  mmu_loading_flag = true;
				if (can_extrude()) mmu_load_step();
				//don't rely on "ok" signal from mmu unit; if filament detected by idler sensor during loading stop loading movements to prevent infinite loading
				if (PIN_GET(IR_SENSOR_PIN) == 0) move = MMU_NO_MOVE;
				break;
			case MMU_UNLOAD_MOVE:
				if (PIN_GET(IR_SENSOR_PIN) == 0) //filament is still detected by idler sensor, printer helps with unlading 
				{
          if (can_extrude())
          {
              printf_P(PSTR("Unload 1\n"));
              current_position[E_AXIS] = current_position[E_AXIS] - MMU_LOAD_FEEDRATE * MMU_LOAD_TIME_MS*0.001;
              plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
              st_synchronize();
          }
				}
				else //filament was unloaded from idler, no additional movements needed 
				{ 
					printf_P(PSTR("Unloading finished 1\n"));
					disable_e0(); //turn off E-stepper to prevent overheating and alow filament pull-out if necessary
					move = MMU_NO_MOVE;
				}
				break;
			case MMU_TCODE_MOVE: //first do unload and then continue with infinite loading movements
				if (PIN_GET(IR_SENSOR_PIN) == 0) //filament detected by idler sensor, we must unload first 
				{
          if (can_extrude())
          {
              printf_P(PSTR("Unload 2\n"));
              current_position[E_AXIS] = current_position[E_AXIS] - MMU_LOAD_FEEDRATE * MMU_LOAD_TIME_MS*0.001;
              plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
              st_synchronize();
          }
				}
				else //delay to allow mmu unit to pull out filament from bondtech gears and then start with infinite loading 
				{ 
					printf_P(PSTR("Unloading finished 2\n"));
					disable_e0(); //turn off E-stepper to prevent overheating and alow filament pull-out if necessary
					delay_keep_alive(MMU_LOAD_TIME_MS);
					move = MMU_LOAD_MOVE;
					get_response_print_info(move);
				}
				break;
			case MMU_NO_MOVE:
			default: 
				delay_keep_alive(100);
				break;
		}
	}
	printf_P(PSTR("mmu_get_response() returning: %d\n"), mmu_ready);
	bool ret = mmu_ready;
	mmu_ready = false;
//	printf_P(PSTR("mmu_get_response - end %d\n"), ret?1:0);
	return ret;
}

//! @brief Wait for active extruder to reach temperature set
//!
//! This function is blocking and showing lcd_wait_for_heater() screen
//! which is constantly updated with nozzle temperature.
void mmu_wait_for_heater_blocking()
{
  while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
  {
    delay_keep_alive(1000);
    lcd_wait_for_heater();
  }
}

void manage_response(bool move_axes, bool turn_off_nozzle, uint8_t move)
{
  bool response = false;
  mmu_print_saved = false;
  bool lcd_update_was_enabled = false;
  float hotend_temp_bckp = degTargetHotend(active_extruder);
  float z_position_bckp = current_position[Z_AXIS];
  float x_position_bckp = current_position[X_AXIS];
  float y_position_bckp = current_position[Y_AXIS];
  uint8_t screen = 0; //used for showing multiscreen messages
  while (!response)
  {
    response = mmu_get_response(move); //wait for "ok" from mmu
    if (!response)
    {               //no "ok" was received in reserved time frame, user will fix the issue on mmu unit
      disable_e0(); // Drop E0 Currents to 0.
      if (!mmu_print_saved)
      { //first occurence, we are saving current position, park print head in certain position and disable nozzle heater
        if (lcd_update_enabled)
        {
          lcd_update_was_enabled = true;
          lcd_update_enable(false);
        }
        st_synchronize();
        mmu_print_saved = true;
        #ifdef OCTO_NOTIFICATIONS_ON
        printf_P(PSTR("// action:mmuAttention\n"));
        #endif // OCTO_NOTIFICATIONS_ON
        printf_P(PSTR("MMU not responding\n"));
        hotend_temp_bckp = degTargetHotend(active_extruder);
        if (move_axes)
        {
          z_position_bckp = current_position[Z_AXIS];
          x_position_bckp = current_position[X_AXIS];
          y_position_bckp = current_position[Y_AXIS];

          //lift z
          current_position[Z_AXIS] += Z_PAUSE_LIFT;
          if (current_position[Z_AXIS] > Z_MAX_POS)
            current_position[Z_AXIS] = Z_MAX_POS;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, active_extruder);
          st_synchronize();

          //Move XY to side
          current_position[X_AXIS] = X_PAUSE_POS;
          current_position[Y_AXIS] = Y_PAUSE_POS;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);
          st_synchronize();
        }
        if (turn_off_nozzle)
        {
          //set nozzle target temperature to 0
          setAllTargetHotends(0);
        }
      }

      //first three lines are used for printing multiscreen message; last line contains measured and target nozzle temperature
      if (screen == 0)
      { //screen 0
        lcd_display_message_fullscreen_P(_i("MMU needs user attention."));
        screen++;
      }
      else
      { //screen 1
        if ((degTargetHotend(active_extruder) == 0) && turn_off_nozzle)
          lcd_display_message_fullscreen_P(_i("Press the knob to resume nozzle temperature."));
        else
          lcd_display_message_fullscreen_P(_i("Fix the issue and then press button on MMU unit."));
        screen = 0;
      }

      lcd_set_degree();
      lcd_set_cursor(0, 4); //line 4
      //Print the hotend temperature (9 chars total) and fill rest of the line with space
      int chars = lcd_printf_P(_N("%c%3d/%d%c"), LCD_STR_THERMOMETER[0], (int)(degHotend(active_extruder) + 0.5), (int)(degTargetHotend(active_extruder) + 0.5), LCD_STR_DEGREE[0]);
      lcd_space(9 - chars);

      //5 seconds delay
      for (uint8_t i = 0; i < 50; i++)
      {
        if (lcd_clicked())
        {
          setTargetHotend(hotend_temp_bckp, active_extruder);
          break;
        }
        delay_keep_alive(100);
      }
    }
    else if (mmu_print_saved)
    {
      printf_P(PSTR("MMU starts responding\n"));
      if (turn_off_nozzle)
      {
        lcd_clear();
        setTargetHotend(hotend_temp_bckp, active_extruder);
        if (((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5))
        {
          lcd_display_message_fullscreen_P(_i("MMU OK. Resuming temperature..."));
          delay_keep_alive(3000);
        }
        while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
        {
          delay_keep_alive(1000);
          lcd_wait_for_heater();
        }
      }
      if (move_axes)
      {
        lcd_clear();
        lcd_display_message_fullscreen_P(_i("MMU OK. Resuming position..."));
        current_position[X_AXIS] = x_position_bckp;
        current_position[Y_AXIS] = y_position_bckp;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);
        st_synchronize();
        current_position[Z_AXIS] = z_position_bckp;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, active_extruder);
        st_synchronize();
      }
      else
      {
        lcd_clear();
        lcd_display_message_fullscreen_P(_i("MMU OK. Resuming..."));
        delay_keep_alive(1000); //delay just for showing MMU OK message for a while in case that there are no xyz movements
      }
    }
  }
  if (lcd_update_was_enabled) lcd_update_enable(true);
  shutdownE0(false); // Reset E0 Currents.
}

//! @brief load filament to nozzle of multimaterial printer
//!
//! This function is used only only after T? (user select filament) and M600 (change filament).
//! It is not used after T0 .. T4 command (select filament), in such case, gcode is responsible for loading
//! filament to nozzle.
//!
void mmu_load_to_nozzle()
{
  st_synchronize();

  bool saved_e_relative_mode = axis_relative_modes[E_AXIS];
  if (!saved_e_relative_mode)
    axis_relative_modes[E_AXIS] = true;
  current_position[E_AXIS] += 7.2f;
  float feedrate = 562;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 14.4f;
  feedrate = 871;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 36.0f;
  feedrate = 1393;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 14.4f;
  feedrate = 871;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
  st_synchronize();
  if (!saved_e_relative_mode)
    axis_relative_modes[E_AXIS] = false;
}

void shutdownE0(bool shutdown)
{
  if (shutdown && ((tmc2130_current_h[E_AXIS] != 0) && (tmc2130_current_r[E_AXIS] != 0)))
  {
    mmuE0BackupCurrents[0] = tmc2130_current_h[E_AXIS];
    mmuE0BackupCurrents[1] = tmc2130_current_r[E_AXIS];
    tmc2130_set_current_h(E_AXIS, 0);
    tmc2130_set_current_r(E_AXIS, 0);
    printf_P(PSTR("E-AXIS Disabled.\n"));
  }
  else if (!shutdown && ((tmc2130_current_h[E_AXIS] == 0) && (tmc2130_current_r[E_AXIS] == 0)))
  {
    tmc2130_set_current_h(E_AXIS, mmuE0BackupCurrents[0]);
    tmc2130_set_current_r(E_AXIS, mmuE0BackupCurrents[1]);
    printf_P(PSTR("E-AXIS Enabled.\n"));
  }
}

void mmu_M600_wait_and_beep()
{
  //Beep and wait for user to remove old filament and prepare new filament for load

  disable_e0();
  float hotend_temp_bckp = degTargetHotend(active_extruder);
  setAllTargetHotends(0);
  KEEPALIVE_STATE(PAUSED_FOR_USER);

  int counterBeep = 0;
  lcd_display_message_fullscreen_P(_i("Remove old filament and press the knob to start loading new filament."));
  bool bFirst = true;

  while (!lcd_clicked())
  {
    manage_heater();
    manage_inactivity(true);

#if BEEPER > 0
    if (counterBeep == 500)
    {
      counterBeep = 0;
    }
    SET_OUTPUT(BEEPER);
    if (counterBeep == 0)
    {
      if ((eSoundMode == e_SOUND_MODE_LOUD) || ((eSoundMode == e_SOUND_MODE_ONCE) && bFirst))
      {
        bFirst = false;
        WRITE(BEEPER, HIGH);
      }
    }
    if (counterBeep == 20)
    {
      WRITE(BEEPER, LOW);
    }

    counterBeep++;
#endif //BEEPER > 0

    delay_keep_alive(4);
  }
  setTargetHotend(hotend_temp_bckp, active_extruder);
  lcd_clear();
  while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
  {
    delay_keep_alive(1000);
    lcd_wait_for_heater();
  }
  WRITE(BEEPER, LOW);
}

void mmu_M600_load_filament(bool automatic, float nozzle_temp)
{
  //load filament for mmu v2
  tmp_extruder = mmu_extruder;
  if (!automatic)
  {
#ifdef MMU_M600_SWITCH_EXTRUDER
    bool yes = lcd_show_fullscreen_message_yes_no_and_wait_P(_i("Do you want to switch extruder?"), false);
    if (yes)
      tmp_extruder = choose_extruder_menu();
#endif //MMU_M600_SWITCH_EXTRUDER
  }
  else
  {
    tmp_extruder = (tmp_extruder + 1) % 5;
  }
  lcd_update_enable(false);
  lcd_clear();
  lcd_set_cursor(0, 1);
  lcd_puts_P(_T(MSG_LOADING_FILAMENT));
  lcd_print(" ");
  lcd_print(tmp_extruder + 1);
  snmm_filaments_used |= (1 << tmp_extruder); //for stop print

  setTargetHotend(nozzle_temp, active_extruder);
  mmu_wait_for_heater_blocking();

  mmu_command(MmuCmd::T0 + tmp_extruder);

  manage_response(false, true, MMU_LOAD_MOVE);
  mmu_continue_loading(is_usb_printing || (lcd_commands_type == LcdCommands::Layer1Cal));
  mmu_extruder = tmp_extruder; //filament change is finished
  mmu_load_to_nozzle();
  load_filament_final_feed();
  st_synchronize();
}

#ifdef SNMM
void extr_mov(float shift, float feed_rate)
{ //move extruder no matter what the current heater temperature is
  set_extrude_min_temp(.0);
  current_position[E_AXIS] += shift;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feed_rate, active_extruder);
  set_extrude_min_temp(EXTRUDE_MINTEMP);
}
#endif //SNMM

void change_extr(int
#ifdef SNMM
                     extr
#endif //SNMM
)
{ //switches multiplexer for extruders
#ifdef SNMM
  st_synchronize();
  delay(100);

  disable_e0();
  disable_e1();
  disable_e2();

  mmu_extruder = extr;

  pinMode(E_MUX0_PIN, OUTPUT);
  pinMode(E_MUX1_PIN, OUTPUT);

  switch (extr)
  {
  case 1:
    WRITE(E_MUX0_PIN, HIGH);
    WRITE(E_MUX1_PIN, LOW);

    break;
  case 2:
    WRITE(E_MUX0_PIN, LOW);
    WRITE(E_MUX1_PIN, HIGH);

    break;
  case 3:
    WRITE(E_MUX0_PIN, HIGH);
    WRITE(E_MUX1_PIN, HIGH);

    break;
  default:
    WRITE(E_MUX0_PIN, LOW);
    WRITE(E_MUX1_PIN, LOW);

    break;
  }
  delay(100);
#endif
}

int get_ext_nr()
{ //reads multiplexer input pins and return current extruder number (counted from 0)
#ifndef SNMM
  return (mmu_extruder); //update needed
#else
  return (2 * READ(E_MUX1_PIN) + READ(E_MUX0_PIN));
#endif
}

void display_loading()
{
  switch (mmu_extruder)
  {
  case 1:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T1));
    break;
  case 2:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T2));
    break;
  case 3:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T3));
    break;
  default:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T0));
    break;
  }
}

void extr_adj(int extruder) //loading filament for SNMM
{
#ifndef SNMM
  MmuCmd cmd = MmuCmd::L0 + extruder;
  if (cmd > MmuCmd::L4)
  {
    printf_P(PSTR("Filament out of range %d \n"), extruder);
    return;
  }
  mmu_command(cmd);

  //show which filament is currently loaded
  lcd_setstatuspgm(_T(WELCOME_MSG));
  lcd_return_to_status();
  lcd_update_enable(true);
  char msg[20];
  sprintf_P(msg, PSTR("MMU Loading Ext:%d"), (extruder + 1));
  //********************
  lcd_setstatus(msg); // 20 Chars

  manage_response(false, false);

  lcd_setstatuspgm(_T(WELCOME_MSG));
  lcd_return_to_status();
#else

  bool correct;
  max_feedrate[E_AXIS] = 80;
//max_feedrate[E_AXIS] = 50;
START:
  lcd_clear();
  lcd_set_cursor(0, 0);
  switch (extruder)
  {
  case 1:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T1));
    break;
  case 2:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T2));
    break;
  case 3:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T3));
    break;
  default:
    lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T0));
    break;
  }
  KEEPALIVE_STATE(PAUSED_FOR_USER);
  do
  {
    extr_mov(0.001, 1000);
    delay_keep_alive(2);
  } while (!lcd_clicked());
  //delay_keep_alive(500);
  KEEPALIVE_STATE(IN_HANDLER);
  st_synchronize();
  //correct = lcd_show_fullscreen_message_yes_no_and_wait_P(MSG_FIL_LOADED_CHECK, false);
  //if (!correct) goto	START;
  //extr_mov(BOWDEN_LENGTH/2.f, 500); //dividing by 2 is there because of max. extrusion length limitation (x_max + y_max)
  //extr_mov(BOWDEN_LENGTH/2.f, 500);
  extr_mov(bowden_length[extruder], 500);
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_puts_P(_T(MSG_LOADING_FILAMENT));
  if (strlen(_T(MSG_LOADING_FILAMENT)) > 18)
    lcd_set_cursor(0, 1);
  else
    lcd_print(" ");
  lcd_print(mmu_extruder + 1);
  lcd_set_cursor(0, 2);
  lcd_puts_P(_T(MSG_PLEASE_WAIT));
  st_synchronize();
  max_feedrate[E_AXIS] = 50;
  lcd_update_enable(true);
  lcd_return_to_status();
  lcdDrawUpdate = 2;
#endif
}

struct E_step
{
  float extrude;   //!< extrude distance in mm
  float feed_rate; //!< feed rate in mm/s
};
static const E_step ramming_sequence[] PROGMEM =
    {
        {1.0, 1000.0 / 60},
        {1.0, 1500.0 / 60},
        {2.0, 2000.0 / 60},
        {1.5, 3000.0 / 60},
        {2.5, 4000.0 / 60},
        {-15.0, 5000.0 / 60},
        {-14.0, 1200.0 / 60},
        {-6.0, 600.0 / 60},
        {10.0, 700.0 / 60},
        {-10.0, 400.0 / 60},
        //{-50.0, 2000.0/60},
};

//! @brief Unload sequence to optimize shape of the tip of the unloaded filament
static void filament_ramming()
{
  for (uint8_t i = 0; i < (sizeof(ramming_sequence) / sizeof(E_step)); ++i)
  {
    current_position[E_AXIS] += pgm_read_float(&(ramming_sequence[i].extrude));
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                     current_position[E_AXIS], pgm_read_float(&(ramming_sequence[i].feed_rate)), active_extruder);
    st_synchronize();
  }
}

//! @brief Unload sequence to optimize shape of the tip of the unloaded filament
void mmu_filament_ramming()
{
  for (uint8_t i = 0; i < (sizeof(ramming_sequence) / sizeof(E_step)); ++i)
  {
    current_position[E_AXIS] += pgm_read_float(&(ramming_sequence[i].extrude));
    plan_buffer_line_curposXYZE(pgm_read_float(&(ramming_sequence[i].feed_rate)), active_extruder);
    st_synchronize();
  }
}

//! @brief Rotate extruder idler to catch filament
//! @par synchronize
//!  * true blocking call
//!  * false non-blocking call
void mmu_load_step(bool synchronize)
{
  current_position[E_AXIS] = current_position[E_AXIS] + MMU_LOAD_FEEDRATE * 0.1;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  if (synchronize)
    st_synchronize();
}

//-//
void extr_unload_()
{
  //if(bFilamentAction)
  if (0)
  {
    bFilamentAction = false;
    extr_unload();
  }
  else
  {
    eFilamentAction = FilamentAction::MmuUnLoad;
    bFilamentFirstRun = false;
    if (target_temperature[0] >= EXTRUDE_MINTEMP)
    {
      bFilamentPreheatState = true;
      mFilamentItem(target_temperature[0], target_temperature_bed);
    }
    //     else menu_submenu(mFilamentMenu);
    else
      mFilamentMenu();
  }
}

//! @brief show which filament is currently unloaded
void extr_unload_view()
{
  lcd_clear();
  lcd_set_cursor(0, 1);
  lcd_puts_P(_T(MSG_UNLOADING_FILAMENT));
  lcd_print(" ");
  if (mmu_extruder == MMU_FILAMENT_UNKNOWN)
    lcd_print(" ");
  else
    lcd_print(mmu_extruder + 1);
}

void extr_unload()
{ //unload just current filament for multimaterial printers
#ifdef SNMM
	float tmp_motor[3] = DEFAULT_PWM_MOTOR_CURRENT;
	float tmp_motor_loud[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;
	uint8_t SilentMode = eeprom_read_byte((uint8_t*)EEPROM_SILENT);
#endif

	if (degHotend0() > EXTRUDE_MINTEMP)
	{
#ifndef SNMM
		st_synchronize();

        menu_submenu(extr_unload_view);

		mmu_filament_ramming();

		mmu_command(MmuCmd::U0);
		// get response
		manage_response(false, true, MMU_UNLOAD_MOVE);

        menu_back();
#else //SNMM

		lcd_clear();
		lcd_display_message_fullscreen_P(PSTR(""));
		max_feedrate[E_AXIS] = 50;
		lcd_set_cursor(0, 0); lcd_puts_P(_T(MSG_UNLOADING_FILAMENT));
		lcd_print(" ");
		lcd_print(mmu_extruder + 1);
		lcd_set_cursor(0, 2); lcd_puts_P(_T(MSG_PLEASE_WAIT));
		if (current_position[Z_AXIS] < 15) {
			current_position[Z_AXIS] += 15; //lifting in Z direction to make space for extrusion
			plan_buffer_line_curposXYZE(25, active_extruder);
		}
		
		current_position[E_AXIS] += 10; //extrusion
		plan_buffer_line_curposXYZE(10, active_extruder);
		st_current_set(2, E_MOTOR_HIGH_CURRENT);
		if (current_temperature[0] < 230) { //PLA & all other filaments
			current_position[E_AXIS] += 5.4;
			plan_buffer_line_curposXYZE(2800 / 60, active_extruder);
			current_position[E_AXIS] += 3.2;
			plan_buffer_line_curposXYZE(3000 / 60, active_extruder);
			current_position[E_AXIS] += 3;
			plan_buffer_line_curposXYZE(3400 / 60, active_extruder);
		}
		else { //ABS
			current_position[E_AXIS] += 3.1;
			plan_buffer_line_curposXYZE(2000 / 60, active_extruder);
			current_position[E_AXIS] += 3.1;
			plan_buffer_line_curposXYZE(2500 / 60, active_extruder);
			current_position[E_AXIS] += 4;
			plan_buffer_line_curposXYZE(3000 / 60, active_extruder);
			/*current_position[X_AXIS] += 23; //delay
			plan_buffer_line_curposXYZE(600 / 60, active_extruder); //delay
			current_position[X_AXIS] -= 23; //delay
			plan_buffer_line_curposXYZE(600 / 60, active_extruder); //delay*/
			delay_keep_alive(4700);
		}
	
		max_feedrate[E_AXIS] = 80;
		current_position[E_AXIS] -= (bowden_length[mmu_extruder] + 60 + FIL_LOAD_LENGTH) / 2;
		plan_buffer_line_curposXYZE(500, active_extruder);
		current_position[E_AXIS] -= (bowden_length[mmu_extruder] + 60 + FIL_LOAD_LENGTH) / 2;
		plan_buffer_line_curposXYZE(500, active_extruder);
		st_synchronize();
		//st_current_init();
		if (SilentMode != SILENT_MODE_OFF) st_current_set(2, tmp_motor[2]); //set back to normal operation currents
		else st_current_set(2, tmp_motor_loud[2]);
		lcd_update_enable(true);
		lcd_return_to_status();
		max_feedrate[E_AXIS] = 50;
#endif //SNMM
	}
	else
	{
		show_preheat_nozzle_warning();
	}
}

//wrapper functions for loading filament
void extr_adj_0()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E0"));
#else
  change_extr(0);
  extr_adj(0);
#endif
}

void extr_adj_1()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E1"));
#else
  change_extr(1);
  extr_adj(1);
#endif
}

void extr_adj_2()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E2"));
#else
  change_extr(2);
  extr_adj(2);
#endif
}

void extr_adj_3()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E3"));
#else
  change_extr(3);
  extr_adj(3);
#endif
}

void extr_adj_4()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E4"));
#else
  change_extr(4);
  extr_adj(4);
#endif
}

void mmu_load_to_nozzle_0()
{
  lcd_mmu_load_to_nozzle(0);
}

void mmu_load_to_nozzle_1()
{
  lcd_mmu_load_to_nozzle(1);
}

void mmu_load_to_nozzle_2()
{
  lcd_mmu_load_to_nozzle(2);
}

void mmu_load_to_nozzle_3()
{
  lcd_mmu_load_to_nozzle(3);
}

void mmu_load_to_nozzle_4()
{
  lcd_mmu_load_to_nozzle(4);
}

void mmu_eject_fil_0()
{
  mmu_eject_filament(0, true);
}

void mmu_eject_fil_1()
{
  mmu_eject_filament(1, true);
}

void mmu_eject_fil_2()
{
  mmu_eject_filament(2, true);
}

void mmu_eject_fil_3()
{
  mmu_eject_filament(3, true);
}

void mmu_eject_fil_4()
{
  mmu_eject_filament(4, true);
}

void load_all()
{
#ifndef SNMM
  enquecommand_P(PSTR("M701 E0"));
  enquecommand_P(PSTR("M701 E1"));
  enquecommand_P(PSTR("M701 E2"));
  enquecommand_P(PSTR("M701 E3"));
  enquecommand_P(PSTR("M701 E4"));
#else
  for (int i = 0; i < 4; i++)
  {
    change_extr(i);
    extr_adj(i);
  }
#endif
}

//wrapper functions for changing extruders
void extr_change_0()
{
  change_extr(0);
  lcd_return_to_status();
}

void extr_change_1()
{
  change_extr(1);
  lcd_return_to_status();
}

void extr_change_2()
{
  change_extr(2);
  lcd_return_to_status();
}

void extr_change_3()
{
  change_extr(3);
  lcd_return_to_status();
}

#ifdef SNMM
//wrapper functions for unloading filament
void extr_unload_all()
{
  if (degHotend0() > EXTRUDE_MINTEMP)
  {
    for (int i = 0; i < 4; i++)
    {
      change_extr(i);
      extr_unload();
    }
  }
  else
  {
    show_preheat_nozzle_warning();
    lcd_return_to_status();
  }
}

//unloading just used filament (for snmm)
void extr_unload_used()
{
  if (degHotend0() > EXTRUDE_MINTEMP)
  {
    for (int i = 0; i < 4; i++)
    {
      if (snmm_filaments_used & (1 << i))
      {
        change_extr(i);
        extr_unload();
      }
    }
    snmm_filaments_used = 0;
  }
  else
  {
    show_preheat_nozzle_warning();
    lcd_return_to_status();
  }
}
#endif //SNMM

void extr_unload_0()
{
  change_extr(0);
  extr_unload();
}

void extr_unload_1()
{
  change_extr(1);
  extr_unload();
}

void extr_unload_2()
{
  change_extr(2);
  extr_unload();
}

void extr_unload_3()
{
  change_extr(3);
  extr_unload();
}

void extr_unload_4()
{
  change_extr(4);
  extr_unload();
}

bool mmu_check_version()
{
  return (mmu_buildnr >= MMU_REQUIRED_FW_BUILDNR);
}

void mmu_show_warning()
{
  printf_P(PSTR("MMU2 firmware version invalid. Required version: build number %d or higher."), MMU_REQUIRED_FW_BUILDNR);
  kill(_i("Please update firmware in your MMU2. Waiting for reset."));
}

void lcd_mmu_load_to_nozzle(uint8_t filament_nr)
{
  if (degHotend0() > EXTRUDE_MINTEMP)
  {
    filament_ramming();

    tmp_extruder = filament_nr;
    lcd_setstatuspgm(_T(WELCOME_MSG));
    lcd_return_to_status();
    lcd_update_enable(true);
    char msg[20];
    sprintf_P(msg, PSTR("MMU Loading Ext:%d"), (tmp_extruder + 1));
    //********************
    lcd_setstatus(msg); // 20 Chars
    mmu_command(MmuCmd::T0 + tmp_extruder);
    manage_response(true, true);
    //mmu_command(MmuCmd::C0);
    mmu_continue_loading(false);
    mmu_extruder = tmp_extruder; //filament change is finished
    mmu_load_to_nozzle();
    load_filament_final_feed();
    st_synchronize();
    lcd_load_filament_color_check();
    lcd_setstatuspgm(_T(WELCOME_MSG));
  }
  else
  {
    show_preheat_nozzle_warning();
  }
}

//! @brief Fits filament tip into heatbreak?
//!
//! If PTFE tube is jammed, this causes filament to be unloaded and no longer
//! being detected by the pulley IR sensor.
//! @retval true Fits
//! @retval false Doesn't fit
static bool can_load()
{
  current_position[E_AXIS] += 60;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  current_position[E_AXIS] -= 52;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  st_synchronize();

  uint_least8_t filament_detected_count = 0;
  const float e_increment = 0.2;
  const uint_least8_t steps = 6.0 / e_increment;
  DEBUG_PUTS_P(PSTR("MMU can_load:"));
  for (uint_least8_t i = 0; i < steps; ++i)
  {
    current_position[E_AXIS] -= e_increment;
    plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
    st_synchronize();
    if (0 == PIN_GET(IR_SENSOR_PIN))
    {
      ++filament_detected_count;
      DEBUG_PUTCHAR('O');
    }
    else
    {
      DEBUG_PUTCHAR('o');
    }
  }
  if (filament_detected_count > steps - 4)
  {
    DEBUG_PUTS_P(PSTR(" succeeded."));
    return true;
  }
  else
  {
    DEBUG_PUTS_P(PSTR(" failed."));
    return false;
  }
}

void mmu_eject_filament(uint8_t filament, bool recover)
{
  if (filament < 5)
  {

    if (degHotend0() > EXTRUDE_MINTEMP)
    {
      st_synchronize();

      {
        LcdUpdateDisabler disableLcdUpdate;
        lcd_clear();
        lcd_set_cursor(0, 1);
        lcd_puts_P(_i("Ejecting filament"));
        filament_ramming();
        mmu_command(MmuCmd::E0 + filament);
        manage_response(false, false, MMU_UNLOAD_MOVE);
        if (recover)
        {
          lcd_show_fullscreen_message_and_wait_P(_i("Please remove filament and then press the knob."));
          mmu_command(MmuCmd::R0);
          manage_response(false, false);
        }
      }
    }
    else
    {
      show_preheat_nozzle_warning();
    }
  }
  else
  {
    puts_P(PSTR("Filament nr out of range!"));
  }
}

//! @brief load more
//!
//! Try to feed more filament from MMU if it is not detected by filament sensor.
//! @retval true Success, filament detected by IR sensor
//! @retval false Failed, filament not detected by IR sensor after maximum number of attempts
static bool load_more()
{
  for (uint8_t i = 0; i < MMU_IDLER_SENSOR_ATTEMPTS_NR; i++)
  {
    if (PIN_GET(IR_SENSOR_PIN) == 0)
      return true;
    DEBUG_PRINTF_P(PSTR("Additional load attempt nr. %d\n"), i);
    mmu_command(MmuCmd::C0);
    manage_response(true, true, MMU_LOAD_MOVE);
  }
  return false;
}

static void increment_load_fail()
{
  uint8_t mmu_load_fail = eeprom_read_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL);
  uint16_t mmu_load_fail_tot = eeprom_read_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT);
  if (mmu_load_fail < 255)
    eeprom_update_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL, mmu_load_fail + 1);
  if (mmu_load_fail_tot < 65535)
    eeprom_update_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT, mmu_load_fail_tot + 1);
}

//! @brief continue loading filament
//! @par blocking
//!  * true blocking - do not return until successful load
//!  * false non-blocking - pause print and return on load failure
//!
//! @startuml
//! [*] --> [*] : !ir_sensor_detected /\n send MmuCmd::C0
//! [*] --> LoadMore
//! LoadMore --> [*] : filament \ndetected
//! LoadMore --> Retry : !filament detected /\n increment load fail
//! Retry --> [*] : filament \ndetected
//! Retry --> Unload : !filament \ndetected
//! Unload --> [*] : non-blocking
//! Unload --> Retry : button \nclicked
//!
//! Retry : Cut filament if enabled
//! Retry : repeat last T-code
//! Unload : unload filament
//! Unload : pause print
//! Unload : show error message
//!
//! @enduml
void mmu_continue_loading(bool blocking)
{
  if (!ir_sensor_detected)
  {
    mmu_command(MmuCmd::C0);
    return;
  }

  bool success = load_more();
  if (success)
    success = can_load();

  enum class Ls : uint_least8_t
  {
    Enter,
    Retry,
    Unload,
  };
  Ls state = Ls::Enter;

  const uint_least8_t max_retry = 2;
  uint_least8_t retry = 0;

  while (!success)
  {
    switch (state)
    {
    case Ls::Enter:
      increment_load_fail();
      // no break
    case Ls::Retry:
      mmu_command(MmuCmd::T0 + tmp_extruder);
      manage_response(true, true, MMU_TCODE_MOVE);
      success = load_more();
      if (success)
        success = can_load();
      ++retry; // overflow not handled, as it is not dangerous.
      if (retry >= max_retry)
        state = Ls::Unload;
      break;
    case Ls::Unload:
      stop_and_save_print_to_ram(0, 0);

      //lift z
      current_position[Z_AXIS] += Z_PAUSE_LIFT;
      if (current_position[Z_AXIS] > Z_MAX_POS)
        current_position[Z_AXIS] = Z_MAX_POS;
      plan_buffer_line_curposXYZE(15, active_extruder);
      st_synchronize();

      //Move XY to side
      current_position[X_AXIS] = X_PAUSE_POS;
      current_position[Y_AXIS] = Y_PAUSE_POS;
      plan_buffer_line_curposXYZE(50, active_extruder);
      st_synchronize();

      mmu_command(MmuCmd::U0);
      manage_response(false, true, MMU_UNLOAD_MOVE);

      setAllTargetHotends(0);
      lcd_setstatuspgm(_i("MMU load failed     ")); ////c=20 r=1

      if (blocking)
      {
        marlin_wait_for_click();
        restore_print_from_ram_and_continue(0);
        state = Ls::Retry;
      }
      else
      {
        mmu_fil_loaded = false; //so we can retry same T-code again
        isPrintPaused = true;
        mmu_command(MmuCmd::W0);
        return;
      }
      break;
    }
  }
}