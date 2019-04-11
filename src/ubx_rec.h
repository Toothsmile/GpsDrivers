/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ubx.h
 *
 * U-Blox protocol definition. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 */

#ifndef UBX_REC_H_
#define UBX_REC_H_

#include "gps_helper.h"
#include "../../definitions.h"
#include "ubx.h"

class GPSDriverUBX_rec : public GPSHelper
{
public:
	GPSDriverUBX_rec(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position,
		     struct satellite_info_s *satellite_info);
	virtual ~GPSDriverUBX_rec();
	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

	int restartSurveyIn();
private:

	/**
	 * Parse the binary UBX packet
	 */
	int parseChar(const uint8_t b);

	/**
	 * Start payload rx
	 */
	int payloadRxInit(void);

	/**
	 * Add payload rx byte
	 */
	int payloadRxAdd(const uint8_t b);
	int payloadRxAddNavSvinfo(const uint8_t b);
	int payloadRxAddMonVer(const uint8_t b);

	/**
	 * Finish payload rx
	 */
	int payloadRxDone(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(const uint8_t);

	/**
	 * Send a message
	 * @return true on success, false on write error (errno set)
	 */
	bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);

	/**
	 * Configure message rate
	 * @return true on success, false on write error
	 */
	bool configureMessageRate(const uint16_t msg, const uint8_t rate);

	/**
	 * Calculate & add checksum for given buffer
	 */
	void calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

	/**
	 * Wait for message acknowledge
	 */
	int waitForAck(const uint16_t msg, const unsigned timeout, const bool report);

	/**
	 * combines the configure_message_rate & wait_for_ack calls
	 * @return true on success
	 */
	inline bool configureMessageRateAndAck(uint16_t msg, uint8_t rate, bool report_ack_error = false);

	/**
	 * Calculate FNV1 hash
	 */
	uint32_t fnv1_32_str(uint8_t *str, uint32_t hval);

	void print_info(void);

	struct vehicle_gps_position_s *_gps_position;
	struct satellite_info_s *_satellite_info;
	uint64_t _last_timestamp_time;
	bool			_configured;
	ubx_ack_state_t		_ack_state;
	bool			_got_posllh;
	bool			_got_velned;
	ubx_decode_state_t	_decode_state;
	uint16_t		_rx_msg;
	ubx_rxmsg_state_t	_rx_state;
	uint16_t		_rx_payload_length;
	uint16_t		_rx_payload_index;
	uint8_t			_rx_ck_a;
	uint8_t			_rx_ck_b;
	gps_abstime		_disable_cmd_last;
	uint16_t		_ack_waiting_msg;
	ubx_buf_t		_buf;
	uint32_t		_ubx_version;
	bool			_use_nav_pvt;
	OutputMode		_output_mode = OutputMode::GPS;

	rtcm_message_t	*_rtcm_message = nullptr;
};

#endif /* UBX_REC_H_ */
