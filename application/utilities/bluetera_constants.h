/*
	MIT License

	Copyright (c) 2019 Tensor Technologies LTD

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef APPLICATION_CONSTANTS_H_
#define APPLICATION_CONSTANTS_H_

// Device information related
#define DEVICE_NAME         "BlueTera"
#define MANUFACTURER_NAME   "Iotera"
#define HARDWARE_VERSION	0x0100			// 2 bytes, will be translated to <MSB>.<LSB> (e.g. 0x0120 --> 1.32)
#define FIRMWARE_VERSION	0x0100			// 2 bytes, will be translated to <MSB>.<LSB>.<commit> (e.g. 1.20.c5f22e0c)

// BLE and internal stack configuration

#define HVN_TX_QUEUE_SIZE	6

#endif /* APPLICATION_CONSTANTS_H_ */
