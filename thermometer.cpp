/*****************************************************************//**
 * @file main_sampler_test.cpp
 *
 * @brief Basic test of nexys4 ddr mmio cores
 *
 * @author p chu
 * @version v1.0: initial release
 *********************************************************************/

// #define _DEBUG
#include "chu_init.h"
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "spi_core.h"
#include "i2c_core.h"
#include "ps2_core.h"
#include "ddfs_core.h"
#include "adsr_core.h"
#include <cmath>
#include <cstdio>
#include <cstring>
using namespace std;

GpoCore led(get_slot_addr(BRIDGE_BASE, S2_LED));
GpiCore sw(get_slot_addr(BRIDGE_BASE, S3_SW));
XadcCore adc(get_slot_addr(BRIDGE_BASE, S5_XDAC));
PwmCore pwm(get_slot_addr(BRIDGE_BASE, S6_PWM));
DebounceCore btn(get_slot_addr(BRIDGE_BASE, S7_BTN));
SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));
SpiCore spi(get_slot_addr(BRIDGE_BASE, S9_SPI));
I2cCore adt7420(get_slot_addr(BRIDGE_BASE, S10_I2C));
Ps2Core ps2(get_slot_addr(BRIDGE_BASE, S11_PS2));
DdfsCore ddfs(get_slot_addr(BRIDGE_BASE, S12_DDFS));
AdsrCore adsr(get_slot_addr(BRIDGE_BASE, S13_ADSR), &ddfs);

int main() {

	//Instantiating pointers
	I2cCore *adt7420_p = &adt7420;
	GpoCore *led_p = &led;

	//Declarations
	const uint8_t DEV_ADDR = 0x4b;
	uint8_t wbytes[2], bytes[2];
	uint16_t tmp;
	int final_decimal, final_int;
	double integral;
	float tmpC,  fractionalPart;
	int first_digit, second_digit, third_digit, fourth_digit;

	//Setting the decimal point
	sseg.set_dp(0x40);
	// read adt7420 id register to verify device existence
	// ack = adt7420_p->read_dev_reg_byte(DEV_ADDR, 0x0b, &id);
	while(1){
		wbytes[0] = 0x0b;
		adt7420_p->write_transaction(DEV_ADDR, wbytes, 1, 1);
		adt7420_p->read_transaction(DEV_ADDR, bytes, 1, 0);
		uart.disp(bytes[0], 16);
		uart.disp("\n\r");
		wbytes[0] = 0x00;
		adt7420_p->write_transaction(DEV_ADDR, wbytes, 1, 1);
		adt7420_p->read_transaction(DEV_ADDR, bytes, 2, 0);

		// conversion
		tmp = (uint16_t) bytes[0];
		tmp = (tmp << 8) + (uint16_t) bytes[1];
		if (tmp & 0x8000) {
			tmp = tmp >> 3;
			tmpC = (float) ((int) tmp - 8192) / 16;
		}
		else {
			tmp = tmp >> 3;
			tmpC = (float) tmp / 16;
		}

		//Splitting the temp sensor output into two parts
		fractionalPart = modf(tmpC, &integral);

		final_decimal = static_cast<int>(fractionalPart * 1000.0);
		final_int = static_cast<int>(tmpC);

		//Splitting the integer values into digits
		first_digit = final_int / 10;
		second_digit = final_int % 10;
		third_digit = final_decimal / 100;
		fourth_digit = final_decimal / 10 % 10;

		//Converting the digits into hex values off of the SSEG class
		first_digit = sseg.h2s(first_digit);
		second_digit = sseg.h2s(second_digit);
		third_digit = sseg.h2s(third_digit);
		fourth_digit = sseg.h2s(fourth_digit);

		//Displaying the Values to the SSEG
		uint8_t sseg_display[] = {0xff, 0xff, 0xff, 0b1000110, fourth_digit, third_digit, second_digit, first_digit};
		sseg.write_8ptn((uint8_t*)sseg_display);

		//UART display
		uart.disp("temperature (C): ");
		uart.disp(tmpC);
		uart.disp("\n\r");
		led_p->write(tmp);
		sleep_ms(1000);
		led_p->write(0);
	}

} //main

