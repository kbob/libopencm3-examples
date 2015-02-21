#include "lcd-spi.h"
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Initialize the ST Micro TFT Display using the SPI port
 */
#include <stdint.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "console.h"
#include "clock.h"
#include "sdram.h"
#include "lcd-spi.h"


// /* forward prototypes for some helper functions */
// static int print_decimal(int v);
// static int print_hex(int v);

/*
 * This is an ungainly workaround (aka hack) basically I want to know
 * when the SPI port is 'done' sending all of the bits out, and it is
 * done when it has clocked enough bits that it would have received a
 * byte. Since we're using the SPI port in write_only mode I am not
 * collecting the "received" bytes into a buffer, but one could of
 * course. I keep track of how many bytes should have been returned
 * by decrementing the 'rx_pend' volatile. When it reaches 0 we know
 * we are done.
 */

volatile int rx_pend;
volatile uint16_t spi_rx_buf;

/*
 * This is the ISR we use. Note that the name is based on the name
 * in the irq.json file of libopencm3 plus the "_isr" extension.
 */
void
spi5_isr(void) {
    spi_rx_buf = SPI_DR(SPI5);
    --rx_pend;
}

/* Simple double buffering, one frame is displayed, the
 * other being built.
 */
uint16_t *cur_frame;
uint16_t *display_frame;


/*
 * Drawing a pixel consists of storing a 16 bit value in the
 * memory used to hold the frame. This code computes the address
 * of the word to store, and puts in the value we pass to it.
 */
void
lcd_draw_pixel(int x, int y, uint16_t color) {
    *(cur_frame + x + y * LCD_WIDTH) = color;
}

/*
 * Fun fact, same SPI port as the MEMS example but different
 * I/O pins. Clearly you can't use both the SPI port and the
 * MEMS chip at the same time in this example.
 *
 * For the STM32-DISCO board, SPI pins in use:
 *  N/C - RESET
 *  PC2 - CS (could be NSS but won't be)
 *  PF7 - SCLK (AF5) SPI5
 *  PD13 - DATA / CMD*
 *  PF9 - MOSI (AF5) SPI5
 */

/*
 * This structure defines the sequence of commands to send
 * to the Display in order to initialize it. The AdaFruit
 * folks do something similar, it helps when debugging the
 * initialization sequence for the display.
 */
struct tft_command {
	uint16_t delay;		// If you need a delay after
	uint8_t cmd;		// command to send
	uint8_t n_args;		// How many arguments it has
};


/* prototype for lcd_command */
static void lcd_command(uint8_t cmd, int delay, int n_args,
						const uint8_t *args);

/*
 * void lcd_command(cmd, delay, args, arg_ptr)
 *
 * All singing all dancing 'do a command' feature. Basically it
 * sends a command, and if args are present it sets 'data' and
 * sends those along too.
 */
static void
lcd_command(uint8_t cmd, int delay, int n_args, const uint8_t *args) {
	uint32_t timeout;
	int i;

	gpio_clear(GPIOC, GPIO2);	// Select the LCD
	rx_pend++;
	spi_send(SPI5, cmd);
	/* We need to wait until it is sent, if we turn on the Data
	 * line too soon, it ends up confusing the display to thinking
	 * its a data transfer, as it samples the D/CX line on the last
	 * bit sent.
	 */
	for (timeout = 0; (timeout < 1000) && (rx_pend); timeout++);
	rx_pend = 0; // sometimes, at 10Mhz we miss this
	if (n_args) {
		gpio_set(GPIOD, GPIO13);	// Set the D/CX pin
		for (i = 0; i < n_args; i++) {
			rx_pend++;
			spi_send(SPI5, *(args+i));
		}
		/* This wait so that we don't pull CS too soon after
		 * sending the last byte of data.
		 */
		for (timeout = 0; (timeout < 1000) && (rx_pend); timeout++);
	}
	gpio_set(GPIOC, GPIO2); 	// Turn off chip select
	gpio_clear(GPIOD, GPIO13);	// always reset D/CX
	if (delay) {
		msleep(delay);	// wait, if called for
	}
}

#define EXPERIMENT 1
/*
 * This creates a 'script' of commands that can be played
 * to the LCD controller to initialize it.
 * One array holds the 'argument' bytes, the other
 * the commands.
 * Keeping them in sync is essential
 */
static const uint8_t cmd_args[] = {

#ifdef EXPERIMENT
    // Begin experiment.
        0xc3, 0x08, 0x50,
        0x00, 0xc1, 0x30,
        0x64, 0x03, 0x12, 0x81,
        0x85, 0x00, 0x78,
        0x39, 0x2c, 0x00, 0x34, 0x02,
        0x20,
        0x00, 0x00,
    // End experiment.
#endif

	0x00, 0x1B,
	0x0a, 0xa2,
	0x10,
	0x10,
	0x45, 0x15,
	0x90,
//    0xc8,                 // original
//                  11001000 = MY, MX, BGR
	0x08,
#ifdef EXPERIMENT
        0xc0,
#else
	0xc2,
	0x55,
#endif
	0x0a, 0xa7, 0x27, 0x04,
	0x00, 0x00, 0x00, 0xef,
	0x00, 0x00, 0x01, 0x3f,
#ifdef EXPERIMENT
        0x01, 0x00, 0x06,         // original
#else
//    0x01, 0x00, 0x06,         // original
	0x01, 0x00, 0x00,           // modified to remove RGB mode
#endif
	0x01,
	0x0F, 0x29, 0x24, 0x0C, 0x0E,
	0x09, 0x4E, 0x78, 0x3C, 0x09,
	0x13, 0x05, 0x17, 0x11, 0x00,
	0x00, 0x16, 0x1B, 0x04, 0x11,
	0x07, 0x31, 0x33, 0x42, 0x05,
	0x0C, 0x0A, 0x28, 0x2F, 0x0F,
};

/*
 * These are the commands we're going to send to the
 * display to initialize it. We send them all, in sequence
 * with occasional delays. Commands that require data bytes
 * as arguments, indicate how many bytes to pull out the
 * above array to include.
 *
 * The sequence was pieced together from the ST Micro demo
 * code, the data sheet, and other sources on the web.
 */
const struct tft_command  initialization[] = {

#ifdef EXPERIMENT
    // Begin experiment.
        {   0, 0xca, 3 },       // 0xc3, 0x08, 0x50,
        {   0, 0xcf, 3 },       // 0x00, 0xc1, 0x30,
        {   0, 0xed, 4 },       // 0x64, 0x03, 0x12, 0x81,
        {   0, 0xe8, 3 },       // 0x85, 0x00, 0x78,
        {   0, 0xcb, 5 },       // 0x39, 0x2c, 0x00, 0x34, 0x02,
        {   0, 0xf7, 1 },       // 0x20,
        {   0, 0xea, 2 },       // 0x00, 0x00,
    // End experiment.
#endif

	{   0, 0xb1, 2 },	// 0x00, 0x1B,
	{   0, 0xb6, 2 },	// 0x0a, 0xa2,
	{   0, 0xc0, 1 },	// 0x10,
	{   0, 0xc1, 1 },	// 0x10,
	{   0, 0xc5, 2 },	// 0x45, 0x15,
	{   0, 0xc7, 1 },	// 0x90,
	{   0, 0x36, 1 },	// 0xc8,
#ifdef EXPERIMENT
	{   0, 0xb0, 1 },	// 0xc0,
#else
	{   0, 0xb0, 1 },	// 0xc2,
	{   0, 0x3a, 1 },	// 0x55 **added, pixel format 16 bpp
#endif
	{   0, 0xb6, 4 },	// 0x0a, 0xa7, 0x27, 0x04,
	{   0, 0x2A, 4 },	// 0x00, 0x00, 0x00, 0xef,
	{   0, 0x2B, 4 },	// 0x00, 0x00, 0x01, 0x3f,
	{   0, 0xf6, 3 },	// 0x01, 0x00, 0x06,
	{ 200, 0x2c, 0 },
	{   0, 0x26, 1},	// 0x01,
	{   0, 0xe0, 15 },	// 0x0F, 0x29, 0x24, 0x0C, 0x0E,
				// 0x09, 0x4E, 0x78, 0x3C, 0x09,
				// 0x13, 0x05, 0x17, 0x11, 0x00,
	{   0, 0xe1, 15 },	// 0x00, 0x16, 0x1B, 0x04, 0x11,
				// 0x07, 0x31, 0x33, 0x42, 0x05,
				// 0x0C, 0x0A, 0x28, 0x2F, 0x0F,
	{ 200, 0x11, 0 },
	{   0, 0x29, 0 },
	{   0,    0, 0 }	// cmd == 0 indicates last command
};

// ILI9341 datasheet, pp 46-49:
//
// RCM[1:0} = 0b10    command 0xb0
// DPI[2:0] = 0b110   command 0x3a
// RIM      = 0       command 0xf6
// PCDIV    = ????    command 0xB6

// Pp 239-240:
// external fosc = DOTCLK / (2 * (PCDIV + 1))

// Command 0x3A: COLMOD: Pixel Format Set  LCD_PIXEL_FORMAT
//            Reset              Cube   Chuck
//  DPI[2:0]   110 (18 bit/pix)   110    101 (16 bit/pix)
//  DBI[2:0]   110 (18 bit/pix)   110    101 (16 bit/pix)

// Command 0xB0: RGB Interface Signal      LCD_RGB_INTERFACE
//            Reset              Cube
//  Bypass:      0 (direct)        1 (memory)
//  RCM[1:0]    10                10
//  VSPL         0 (low)           0
//  HSPL         0 (low)           0
//  DPL          0 (rising)        1 (falling)
//  EPL          1 (low)           0 (high)

// Command 0xB6: Display Function Control  LCD_DFC
//            Reset              Cube 0A A7 27 04
//  PTG[1:0]    10                10
//  PT[1:0]     10                10
//  REV          1                 1
//  GS           0                 0
//  SS           0 (S1->S720)      1 (S720->S1)
//  SM           0                 0
//  ISC[3:0]  0010 (5 frames)   0111 (15 frames)
//  NL[5:0]   100111          100111
//  PCDIV[5:0]   ?            000100


// Command 0xF6: Interface Control         LCD_INTERFACE
//            Reset              Cube  01 00 06
//  MY_EOR       0                 0
//  MX_EOR       0                 0
//  MV_EOR       0                 0
//  BGR_EOR      0                 0
//  WEMODE       1 (wrap)          1
//  EPF[1:0]    00                00
//  MDT[1:0]    00                00
//  ENDIAN       0 (MSB first)     0
//  DM[1:0]     00 (int clk)      01 (RGB ifc)
//  RM           0 (sys ifc)       1 (RGB ifc)
//  RIM          0 (1 xfr/pix)     0

/* prototype for initialize_display */
static void initialize_display(const struct tft_command cmds[]);

/*
 * void initialize_display(struct cmds[])
 *
 * This is the function that sends the entire list. It also puts
 * the commands it is sending to the console.
 */
static void
initialize_display(const struct tft_command cmds[]) {

	int i = 0;
	int arg_offset = 0;

	/* Initially arg offset is zero, so each time we 'consume'
	 * a few bytes in the args array the offset is moved and
	 * that changes the pointer we send to the command function.
	 */
	while (cmds[i].cmd) {
		// console_puts("CMD: ");
		// print_hex(cmds[i].cmd);
		// console_puts(", ");
		// if (cmds[i].n_args) {
		// 	console_puts("ARGS: ");
		// 	for (j = 0; j < cmds[i].n_args; j++) {
		// 		print_hex(cmd_args[arg_offset+j]);
		// 		console_puts(", ");
		// 	}
		// }
		// console_puts("DELAY: ");
		// print_decimal(cmds[i].delay);
		// console_puts("ms\n");

		lcd_command(cmds[i].cmd, cmds[i].delay, cmds[i].n_args,
			&cmd_args[arg_offset]);
		arg_offset += cmds[i].n_args;
		i++;
	}
	// console_puts("Done.\n");
}

/* prototype for test_image */
static void test_image(void);

/*
 * Interesting questions:
 *   - How quickly can I write a full frame?
 *      * Take the bits sent (16 * width * height)
 *        and divide by the  baud rate (10.25Mhz)
 *      * Tests in main.c show that yes, it taks 74ms.
 *
 * Create non-random data in the frame buffer. In our case
 * a black background and a grid 16 pixels x 16 pixels of
 * white lines. No line on the right edge and bottom of screen.
 */
static void
test_image(void) {
	int		x, y;
	uint16_t	pixel;

	for (x = 0; x < LCD_WIDTH; x++) {
		for (y = 0; y < LCD_HEIGHT; y++) {
			pixel = 0;              // all black
			if ((x % 16) == 0) {
				pixel = 0xffff;     // all white
			}
			if ((y % 16) == 0) {
				pixel = 0xffff;     // all white
			}
			lcd_draw_pixel(x, y, pixel);
		}
	}
}

/*
 * void lcd_show_frame(void)
 *
 * Dump an entire frame to the LCD all at once. In theory you
 * could call this with DMA but that is made more difficult by
 * the implementation of SPI and the modules interpretation of
 * D/CX line.
 */
void lcd_show_frame(void) {
	uint16_t	*t;
	uint8_t size[4];

	t = display_frame;
	display_frame = cur_frame;
	cur_frame = t;
	/*  */
	size[0] = 0;
	size[1] = 0;
	size[2] = (LCD_WIDTH >> 8) & 0xff;
	size[3] = (LCD_WIDTH) & 0xff;
	lcd_command(0x2A, 0, 4, (const uint8_t *)&size[0]);
	size[0] = 0;
	size[1] = 0;
	size[2] = (LCD_HEIGHT >> 8) & 0xff;
	size[3] = LCD_HEIGHT & 0xff;
	lcd_command(0x2B, 0, 4, (const uint8_t *)&size[0]);
	lcd_command(0x2C, 0, FRAME_SIZE_BYTES, (const uint8_t *)display_frame);
}

/*
 * void lcd_spi_init(void)
 *
 * Initialize the SPI port, and the through that port
 * initialize the LCD controller. Note that this code
 * will expect to be able to draw into the SDRAM on
 * the board, so the sdram much be initialized before
 * calling this function.
 *
 * SPI Port and GPIO Defined - for STM32F4-Disco
 *
 * LCD_CS      PC2
 * LCD_SCK     PF7
 * LCD_DC      PD13
 * LCD_MOSI    PF9
 * LCD_SPI     SPI5
 * LCD_WIDTH   240
 * LCD_HEIGHT  320
 */
void
lcd_spi_init(void) {
	uint32_t	tmp;

	/*
	 * Set up the GPIO lines for the SPI port and
	 * control lines on the display.
	 */
	rcc_periph_clock_enable(RCC_GPIOC | RCC_GPIOD | RCC_GPIOF);

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO9);

	cur_frame = (uint16_t *)(SDRAM_BASE_ADDRESS);
	display_frame = cur_frame + (LCD_WIDTH * LCD_HEIGHT);

	rx_pend = 0;
	/* Implement state management hack */
	nvic_enable_irq(NVIC_SPI5_IRQ);

	rcc_periph_clock_enable(RCC_SPI5);
	/* This should configure SPI5 as we need it configured */
	tmp = SPI_SR(LCD_SPI);
	SPI_CR2(LCD_SPI) |= (SPI_CR2_SSOE | SPI_CR2_RXNEIE);

	/* device clocks on the rising edge of SCK with MSB first */
	tmp = SPI_CR1_BAUDRATE_FPCLK_DIV_4 |	// 10.25Mhz SPI Clock (42M/4)
		SPI_CR1_MSTR |                  // Master Mode
		SPI_CR1_BIDIOE |                // Write Only
		SPI_CR1_SPE;                    // Enable SPI

	SPI_CR1(LCD_SPI) = tmp;                 // Do it.
	if (SPI_SR(LCD_SPI) & SPI_SR_MODF) {
		SPI_CR1(LCD_SPI) = tmp;		// Re-writing will reset MODF
		console_puts("Initial mode fault.\n");
	}

	/* Set up the display */
	// console_puts("Initialize the display.\n");
	initialize_display(initialization);

	/* create a test image */
	// console_puts("Generating Test Image\n");
	test_image();

	/* display it on the LCD */
	// console_puts("And ... voila\n");
 	// lcd_show_frame();
}

// /*
//  * int len = print_decimal(int value)
//  *
//  * Very simple routine to print an integer as a decimal
//  * number on the console.
//  */
// int
// print_decimal(int num) {
// 	int	ndx = 0;
// 	char	buf[10];
// 	int	len = 0;
// 	char	is_signed = 0;

// 	if (num < 0) {
// 		is_signed++;
// 		num = 0 - num;
// 	}
// 	buf[ndx++] = '\000';
// 	do {
// 		buf[ndx++] = (num % 10) + '0';
// 		num = num / 10;
// 	} while (num != 0);
// 	ndx--;
// 	if (is_signed != 0) {
// 		console_putc('-');
// 		len++;
// 	}
// 	while (buf[ndx] != '\000') {
// 		console_putc(buf[ndx--]);
// 		len++;
// 	}
// 	return len; // number of characters printed
// }

// /*
//  * int print_hex(int value)
//  *
//  * Very simple routine for printing out hex constants.
//  */
// static int print_hex(int v) {
// 	int		ndx = 0;
// 	char	buf[10];
// 	int		len;

// 	buf[ndx++] = '\000';
// 	do {
// 		char	c = v & 0xf;
// 		buf[ndx++] = (c > 9) ? '7'+ c : '0' + c;
// 		v = (v >> 4) & 0x0fffffff;
// 	} while (v != 0);
// 	ndx--;
// 	console_puts("0x");
// 	len = 2;
// 	while (buf[ndx] != '\000') {
// 		console_putc(buf[ndx--]);
// 		len++;
// 	}
// 	return len; // number of characters printed
// }
