/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef OPEN_DRONE_ID_BASIC_ID_HPP
#define OPEN_DRONE_ID_BASIC_ID_HPP

#include <stdio.h>
#include <string.h>

#include <px4_platform_common/board_common.h>
#include <uORB/topics/vehicle_status.h>
#include <modules/mavlink/open_drone_id_translations.hpp>

class MavlinkStreamOpenDroneIdBasicId : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdBasicId(mavlink); }

	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_BASIC_ID"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _vehicle_status_sub.advertised() ? MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamOpenDroneIdBasicId(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};



	bool send() override
	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			mavlink_open_drone_id_basic_id_t msg{};

			msg.target_system = 0; // 0 for broadcast
			msg.target_component = 0; // 0 for broadcast
			// msg.id_or_mac // Only used for drone ID data received from other UAs.

			// id_type: MAV_ODID_ID_TYPE
			msg.id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER;

			// ua_type: MAV_ODID_UA_TYPE
			msg.ua_type = open_drone_id_translations::odidTypeForMavType(vehicle_status.system_type);

			// uas_id: UAS (Unmanned Aircraft System) ID per GB 46860-2025:
			//         4-digit manufacturer code + 4-digit product model code + 12-digit serial number = 20 digits,
			//         e.g. "TFHKTEST000000000001". The 20 chars fill the 20-byte field completely (no trailing null).
			//         Manufacturer/model identify the product; the serial is derived from the board UID
			//         (FNV-1a hash over the GUID, reduced mod 1e12) so each unit gets a unique ID.
			static constexpr char kGbidManufacturerCode[] = "TFHK"; // TODO: change to your GB 46860-2025 manufacturer code
			static constexpr char kGbidModelCode[] = "TEST"; // TODO: change to your product model code

			uint64_t serial_hash = 1469598103934665603ULL;

#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);

			for (unsigned i = 0; i < PX4_GUID_BYTE_LENGTH; i++) {
				serial_hash ^= px4_guid[i];
				serial_hash *= 1099511628211ULL;
			}

#else
			/* Targets without a hardware GUID (e.g. SITL) have no px4_guid_t; derive the serial
			 * from the 32-bit UUIDs the platform does provide. */
			uuid_uint32_t uuid;
			board_get_uuid32(uuid);

			for (unsigned i = 0; i < sizeof(uuid); i++) {
				serial_hash ^= reinterpret_cast<const uint8_t *>(uuid)[i];
				serial_hash *= 1099511628211ULL;
			}

#endif

			char gb_uaid[4 + 4 + 12 + 1];
			snprintf(gb_uaid, sizeof(gb_uaid), "%s%s%012llu",
				 kGbidManufacturerCode, kGbidModelCode,
				 (unsigned long long)(serial_hash % 1000000000000ULL));

			memcpy(msg.uas_id, gb_uaid, sizeof(msg.uas_id));

			mavlink_msg_open_drone_id_basic_id_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_BASIC_ID_HPP
