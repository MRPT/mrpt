/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include "hal/abs_rxtx.h"

namespace rp::arch::net
{
class raw_serial : public rp::hal::serial_rxtx
{
   public:
	enum
	{
		SERIAL_RX_BUFFER_SIZE = 512,
		SERIAL_TX_BUFFER_SIZE = 128,
	};

	raw_serial();
	~raw_serial() override;
	bool bind(
		const char* portname, uint32_t baudrate, uint32_t flags = 0) override;
	bool open() override;
	void close() override;
	void flush(_u32 flags) override;

	int waitfordata(
		size_t data_count, _u32 timeout = -1,
		size_t* returned_size = nullptr) override;

	int senddata(const unsigned char* data, size_t size) override;
	int recvdata(unsigned char* data, size_t size) override;

	int waitforsent(
		_u32 timeout = -1, size_t* returned_size = nullptr) override;
	int waitforrecv(
		_u32 timeout = -1, size_t* returned_size = nullptr) override;

	size_t rxqueue_count() override;

	void setDTR() override;
	void clearDTR() override;

	_u32 getTermBaudBitmap(_u32 baud);

   protected:
	bool open(const char* portname, uint32_t baudrate, uint32_t flags = 0);
	void _init();

	char _portName[200];
	uint32_t _baudrate{0};
	uint32_t _flags{0};

	int serial_fd{-1};

	size_t required_tx_cnt;
	size_t required_rx_cnt;
};
}  // namespace rp::arch::net
