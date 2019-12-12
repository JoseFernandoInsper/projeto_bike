/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "tfont.h"
#include "relogio.h"
#include "relogioI.h"
#include "acelerometro.h"
#include "desacelerometro.h"
#include "igualometro.h"
#include "kmh.h"
#include "pause.h"
#include "play.h"
#include "restart.h"
#include "comic.h"
#include "kmzin.h"
#include "VelInst.h"

#define YEAR        2019
#define MOUNT       9
#define DAY         12
#define WEEK        46
#define HOUR        0
#define MINUTE      13
#define SECOND      45

#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      19
#define BUT_IDX_MASK (1 << BUT_IDX)

#define KMH_X 185
#define KMH_Y 162
#define ACELEROMETRO_X 10
#define ACELEROMETRO_Y 100
#define RELOGIO_X 168
#define RELOGIO_Y 2
#define RELOGIO_I_X 135
#define RELOGIO_I_Y 4 
#define PAUSE_X 80
#define PAUSE_Y 400
#define RESTART_X 200
#define RESTART_Y 400
#define raio 1
#define BACKGROUND_COLOR 0xADFFA6u

#define MAX_ENTRIES        30
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

volatile bool time_flag = false;
volatile bool tempo_flag = false;
volatile bool pause_flag = false;
volatile bool flag_pulso = false;
volatile bool w_flag = false;
volatile int Time_g = 0;
volatile int dist_g = 0;
volatile int pulse_medio = 0;
volatile int pulse_instantaneo = 0;
volatile int aceleracao = 1;


/************************************************************************/
/* prototype                                                            */
/************************************************************************/
uint32_t convert_axis_system_x(uint32_t touch_y);
uint32_t convert_axis_system_y(uint32_t touch_x);
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing);
void TimerInc(uint *Time_g);
void update_screen_button(uint32_t tx, uint32_t ty);

/************************************************************************/
/* handler / callback                                                   */
/************************************************************************/

void but_callback(void)
{
	flag_pulso = true;
	if(!pause_flag){
		pulse_medio++;}
	pulse_instantaneo++;
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	unsigned int hora;
	unsigned int minuto;
	unsigned int segundo;
	

	rtc_get_time(RTC, &hora, &minuto , &segundo);
	
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		if(pause_flag == false){
			Time_g++;
			tempo_flag = true;
		}
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	
	time_flag = true;
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void mxt_handler(struct mxt_device *device, uint32_t *x, uint32_t *y)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		if(i==0){
			 // eixos trocados (quando na vertical LCD)
			*x = convert_axis_system_x(touch_event.y);
			*y = convert_axis_system_y(touch_event.x);
		
			/* Format a new entry in the data string that will be sent over USART */
			//sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
			//		touch_event.id, touch_event.x, touch_event.y,
			//		touch_event.status, conv_x, conv_y);
			//update_screen_button(conv_x, conv_y);
		}

		/* Add the new string to the string buffer */
		//strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
	
	

	
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/
int w(){
	int w = pulse_instantaneo/5;
	pulse_instantaneo = 0;
	w_flag = false;
	return w;
}
int a(void) {
	return 0;//K * (x[N] - x[N-1])/Ts;
}

int v(void) {
	return w()*raio;
}

int dist(){
	return pulse_medio*6*raio;
}

void TimerInc(uint *Time_g) {
	Time_g++;
}

int calcH(void){
	int timeee = Time_g/3600;
	return timeee;
}

int calcM(void){
	int timee = Time_g/60;
	if (timee > 59) {
		return timee%60;
	}
	return timee;
}

int calcS(void){
	int templo = Time_g%60; 
	return templo;
}

void pauseF(){
	if(pause_flag == true){
		ili9488_draw_pixmap(PAUSE_X, PAUSE_Y, pause.width, pause.height, pause.data);
		pause_flag =false;
	} else{
		ili9488_draw_pixmap(PAUSE_X, PAUSE_Y, play.width, play.height, play.data);
		pause_flag = true;
	}
	
}

void restartB(){
	Time_g = 0;
	pulse_medio = 0;
}

int calcAce(int old, int new){
	int result = new - old;
	
	if (result < -3){
		aceleracao = 0;
	}
	else if (result > 3){
		aceleracao = 2;
	}
	else {
		aceleracao = 1;
	}
	return new;
}

/************************************************************************/
/* tela                                                                 */
/************************************************************************/

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void update_screen_button(uint32_t tx, uint32_t ty){
	if(tx >= PAUSE_X && tx <= PAUSE_X+53) {
		if(ty >= PAUSE_Y && ty <= PAUSE_Y+69) {
			pauseF();
		} 
	}
	if(tx >= RESTART_X && tx <= RESTART_X+53) {
		if(ty >= RESTART_Y && ty <= RESTART_Y+69) {
			restartB();
		}
	}
}

void update_screen_relogio() {
	unsigned int hora;
	unsigned int minuto;
	unsigned int segundo;
	rtc_get_time(RTC, &hora, &minuto , &segundo);
	char buff[64];
	sprintf(buff, "%02d:%02d:%02d", hora, minuto, segundo);
	
	font_draw_text(&relogio, buff, RELOGIO_X, RELOGIO_Y, 0);
	

}

void update_screen_tempo() {
	int horas = calcH();
	int minutos = calcM();
	int segundos = calcS();
	char buff[64];
	sprintf(buff, "%02d:%02d:%02ds", horas, minutos, segundos);
	font_draw_text(&kmzin, buff, 175, 340, 0);
}

void update_screen_distancia() {
	int oi = dist();
	char buff[64];
	sprintf(buff, "%06d", oi);
	font_draw_text(&kmzin, buff, 175, 300, 0);
}

void update_screen_vel_med() {
	int velmed = 4*dist()/Time_g;
	char buff[64];
	sprintf(buff, "%04d", velmed);
	font_draw_text(&kmzin, buff, 175, 260, 0);
}

double update_screen_vel_insta(double T) {
	double oioi = 10/(double)T;
	int vel = oioi;
	char buff[64];
	sprintf(buff, "%lf", oioi);
	font_draw_text(&VelInst, buff, 170, 100, 0);
	return oioi;
}

void update_screen_aceleration() {
	if (aceleracao == 2){
		ili9488_draw_pixmap(ACELEROMETRO_X, ACELEROMETRO_Y, acelerometro.width, acelerometro.height, acelerometro.data);
	}
	if (aceleracao == 1){
		ili9488_draw_pixmap(ACELEROMETRO_X, ACELEROMETRO_Y, igualometro.width, igualometro.height, igualometro.data);
	}
	if (aceleracao == 0){
		ili9488_draw_pixmap(ACELEROMETRO_X, ACELEROMETRO_Y, desacelerometro.width, desacelerometro.height, desacelerometro.data);
	}
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(BACKGROUND_COLOR));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void draw_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
	if(clicked) {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
		} else {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
	}
	last_state = clicked;
}

void init_tela(void){
	draw_screen();
	//draw_button(0);
	ili9488_draw_pixmap(RELOGIO_I_X, RELOGIO_I_Y, relogioI.width, relogioI.height, relogioI.data);
	ili9488_draw_pixmap(ACELEROMETRO_X, ACELEROMETRO_Y, igualometro.width, igualometro.height, igualometro.data);
	font_draw_text(&KMH, "Km/h", KMH_X, KMH_Y, 0);
	font_draw_text(&comic, "Vel. Media:", 10, 260, 0);
	font_draw_text(&comic, "Distancia:", 10, 300, 0);
	font_draw_text(&comic, "Tempo:", 10, 340, 0);
	font_draw_text(&kmzin, "km/h", 240, 260, 0);
	font_draw_text(&kmzin, "m", 276, 300, 0);
	//font_draw_text(&kmzin, "HH:MM:SS", 175, 340, 0);
	ili9488_draw_pixmap(RESTART_X, RESTART_Y, restart.width, restart.height, restart.data);
	ili9488_draw_pixmap(PAUSE_X, PAUSE_Y, pause.width, pause.height, pause.data);
	
}

/************************************************************************/
/* periferico                                                           */
/************************************************************************/

static void RTT_init(uint16_t pllPreScale)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNT, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_SECEN);

}

void init_perifericos(void){
	RTC_init();
	
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_RISE_EDGE, but_callback);
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
}

/************************************************************************/
/* uteis                                                                */
/************************************************************************/

int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	init_perifericos();

	/* Initialize the mXT touch device */
	mxt_init(&device);
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	printf("\n\rmaXTouch data USART transmitter\n\r");
	init_tela();
	
	uint32_t x,y; 
	double pulsoTime;
	uint32_t timeOld = 0;
	uint32_t timeNow = 0;
	uint16_t pllPreScale = (int) ( 32768.0 / 2);
	
	while (true) {
		if (flag_pulso == true){
			pulsoTime = rtt_read_timer_value(RTT);
			RTT_init(pllPreScale);		
			flag_pulso = false;
			timeOld = calcAce(timeOld, timeNow);
		}
		timeNow = update_screen_vel_insta(pulsoTime/2);
		update_screen_aceleration();
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device, &x, &y);
			update_screen_button(x, y);
		}
		if(time_flag == true){
			update_screen_relogio();
			time_flag = false;
		}
		if (tempo_flag == true) {
			update_screen_tempo();
			update_screen_distancia();
			update_screen_vel_med();
		}

		
		
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		//if (mxt_is_message_pending(&device)) {
		//	S(&device);
		//}
		
	}

	return 0;
}
