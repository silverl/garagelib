#include <cstdint>
#include <secplus.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "garagelib.h"

// Debug callback support - allows runtime-configurable debug output
static garagelib_debug_callback_t _debug_callback = NULL;

extern "C" void garagelib_set_debug_callback(garagelib_debug_callback_t callback) {
	_debug_callback = callback;
}

// Internal debug buffer for formatting messages
static char _debug_buf[128];

static void _debug_print(const char* msg) {
	if (_debug_callback) {
		_debug_callback(msg);
	}
}

// Helper to copy flash string to buffer and print
static void _debug_println_P(const char* flash_str) {
	if (_debug_callback) {
		strncpy_P(_debug_buf, flash_str, sizeof(_debug_buf) - 1);
		_debug_buf[sizeof(_debug_buf) - 1] = '\0';
		_debug_print(_debug_buf);
	}
}

#define GARAGELIB_PRINTLN(x) _debug_println_P((const char*)x)

// Enable detailed packet logging when callback is set
#define GARAGELIB_DEBUG_ENABLED (_debug_callback != NULL)

namespace SecPlusCommon {
	static const size_t metadata_size = 2;
	static SoftwareSerial *sw_serial = NULL;
	static void delete_sw_serial() {
		if(sw_serial != NULL) {
			sw_serial->end();
			delete sw_serial;
			sw_serial = NULL;
		}
	}
	#define NEW_COMMAND_BUFFER(NAME, PACKET, CAP) uint8_t NAME[(PACKET+SecPlusCommon::metadata_size)*CAP]

	enum class DoorStatus : uint8_t {
		UNKNOWN = 0,
		OPEN,
		CLOSED,
		STOPPED,
		OPENING,
		CLOSING,
	};

	class CommandBuffer {
		public:
			CommandBuffer(const size_t packet_size, const size_t buffer_capacity, uint8_t *buffer) {
				this->packet_size = packet_size + metadata_size;
				this->buffer_capacity = buffer_capacity;
				this->buffer = buffer;
			}

			uint8_t get_size() {
				return size;
			}

			uint8_t *get_head() {
				return buffer + (head_index * packet_size) + metadata_size;
			}

			uint16_t get_delay() {
				uint8_t *delay_b = (buffer + (head_index * packet_size));
				return (((uint16_t) *delay_b) << 8) | *(delay_b+1);
			}

			void pop() {
				head_index = (head_index + 1) % buffer_capacity;
				size -= 1;
			}

			uint8_t *push_next() {
				if (size == buffer_capacity) {
					return nullptr;
				} else {
					uint8_t next_index = (head_index + size) % buffer_capacity;
					uint8_t *b = buffer + (next_index * packet_size);
					size += 1;
					return b;
				}
			}
		private:
			uint8_t head_index = 0;
			uint8_t size = 0;
			uint8_t *buffer;

			uint8_t packet_size;
			size_t buffer_capacity;
	};
}

namespace SecPlus2 {
	enum class Command : uint16_t {
		UNKNOWN = 0x000,
		GET_STATUS = 0x080,
		STATUS = 0x081,
		OBST_1 = 0x084,
		OBST_2 = 0x085,
		BATTERY_STATUS = 0x09d,
		PAIR_3 = 0x0a0,
		PAIR_3_RESP = 0x0a1,
		LEARN = 0x181,
		LOCK = 0x18c,
		DOOR_ACTION = 0x280,
		LIGHT = 0x281,
		MOTOR_ON = 0x284,
		MOTION = 0x285,
		GET_PAIRED_DEVICES = 0x307,
		PAIRED_DEVICES = 0x308,
		CLEAR_PAIRED_DEVICES = 0x30D,
		LEARN_1 = 0x391,
		PING = 0x392,
		PING_RESP = 0x393,
		PAIR_2 = 0x400,
		PAIR_2_RESP = 0x401,
		SET_TTC = 0x402,
		CANCEL_TTC = 0x408,
		TTC = 0x40a,
		GET_OPENINGS = 0x48b,
		OPENINGS = 0x48c,
		MAX,
	};

	enum class DoorAction : uint8_t {
		CLOSE = 0,
		OPEN = 1,
		TOGGLE = 2,
		STOP = 3,
	};

	const size_t SEC2_PACKET_SIZE = 19;
	const size_t COMMAND_BUFFER_CAPACITY = 10;
	const uint32_t SEC2_PREAMBLE = 0x00550100;

	class SecPlusReader {
		public:
			SecPlusReader() {}

			uint8_t* get_packet() {
				return packet;
			}

			bool read_byte(uint8_t b) {
				if (index == 0) {
					// If scanning for the preamble shift the current result and add the new part
					preamble <<= 8;
					preamble |= b;
					preamble &= 0x00FFFFFF;

					// If preamble matches start to read rest of packet
					if (preamble == SEC2_PREAMBLE) {
						index = 3;
					}

					return false;
				} else {
					packet[index] = b;
					index += 1;

					// After reading packet reset and return true (ready to use)
					if (index == SEC2_PACKET_SIZE) {
						index = 0;
						preamble = 0;

						return true;
					} else {
						return false;
					}
				}
			}

		private:
			uint8_t packet[SEC2_PACKET_SIZE] = {0x55, 0x01, 0x00};
			uint8_t index = 0;
			uint32_t preamble = 0;
	};

	const size_t FAILED_COMMAND_DELAY = 100;
	const size_t SYNC_DELAY = 1000;

	typedef struct {
		SecPlusCommon::DoorStatus door_state;
		bool light_state;
		bool lock_state;
		bool obstruction_state;
		bool motor_state;
		bool button_state;
		bool battery_state;
		bool learn_state;
		uint16_t openings;
	} state_struct_t;

	typedef void (*state_callback_t)(state_struct_t state);

	class Garage {
		public:
			Garage(uint32_t client_id, int rx_pin, int tx_pin, bool check_collision = true) {
				this->client_id = client_id;
				this->check_collision = check_collision;
				this->rx_pin = rx_pin;
				this->tx_pin = tx_pin;
				this->reader = SecPlusReader();
			}

			void begin() {
				SecPlusCommon::delete_sw_serial();
				SecPlusCommon::sw_serial = new SoftwareSerial();
				// Start Serial with 8 bits no parity 1 stop bit and inverted
				SecPlusCommon::sw_serial->begin(9600, SWSERIAL_8N1, rx_pin, tx_pin, true);
				SecPlusCommon::sw_serial->enableIntTx(false);
			}

			void enable_callback(state_callback_t callback) {
				this->state_callback = callback;
			}

			int8_t request_status() {
				return queue_command(Command::GET_STATUS, 0, 0, 100);
			}

			int8_t request_openings() {
				return queue_command(Command::GET_OPENINGS, 0, 0, 100);
			}

			int8_t set_lock(bool lock_status) {
				// Lock status should be the 1st and 2nd bits of data, 00 and 01 are off and on
				return queue_command(Command::LOCK, lock_status, 0, 100);
			}

			int8_t toggle_lock() {
				// Lock status should be the 1st and 2nd bits of data, 10 is toggle
				return queue_command(Command::LOCK, 0b10, 0, 100);
			}

			int8_t set_light(bool light_status) {
				// Light status should be the 1st and 2nd bits of data, 00 and 01 are off and on
				return queue_command(Command::LIGHT, light_status, 0, 100);
			}

			int8_t toggle_light() {
				// Light status should be the 1st and 2nd bits of data, 10 is toggle
				return queue_command(Command::LIGHT, 0b10, 0, 100);
			}

			int8_t set_door(DoorAction door_action) {
				// Door action is in the 1st and 2nd bits of data
				// Pressed bit is the 5th bit (1st bit of the high part) which needs to be high to work
				// (though other implementations send a high then a low)
				uint16 data_high = 0;

				// For now door id is 1
				uint16 door_id = 1;
				// Door id is the 12th and 13th bits (8th and 9st bits of the high part)
				data_high |= door_id << 8;

				// First send a pressed then a releaase
				int8_t err = queue_command(Command::DOOR_ACTION, static_cast<uint8_t>(door_action), data_high+1, 250);
				if (err < 0) return err;
				return queue_command(Command::DOOR_ACTION, static_cast<uint8_t>(door_action), data_high, 40);
			}

			int8_t open_door() {
				return set_door(DoorAction::OPEN);
			}

			int8_t close_door() {
				return set_door(DoorAction::CLOSE);
			}

			int8_t toggle_door() {
				return set_door(DoorAction::TOGGLE);
			}

			int8_t stop_door() {
				return set_door(DoorAction::STOP);
			}

			void loop() {
				if(!SecPlusCommon::sw_serial) return;

				if (!SecPlusCommon::sw_serial->available()) {
					if (state.door_state == SecPlusCommon::DoorStatus::UNKNOWN && (long)(millis() - next_sync_time) > 0) {
						request_status();
						request_openings();
						next_sync_time = millis() + SYNC_DELAY;
					}

					if (ask_for_status) {
						ask_for_status = false;
						request_status();
					}

					if (buf.get_size() && (long)(millis() - next_command_time) > 0) {
						if (!send_data(buf.get_head())) {
							// No error then pop the head off as the command was successfully sent.
							process_packet(buf.get_head());
							next_command_time = millis() + buf.get_delay();
							buf.pop();
						} else {
							GARAGELIB_PRINTLN(F("Command failed to send"));
							next_command_time = millis() + FAILED_COMMAND_DELAY;
						}
					}
				} else {
					if (reader.read_byte(SecPlusCommon::sw_serial->read())) {
						// If the packet is complete
						uint8_t *packet = reader.get_packet();
						if (GARAGELIB_DEBUG_ENABLED) {
							uint32_t r = 0;
							uint64_t i = 0;
							uint16_t c = 0;
							uint32_t p = 0;
							decode_wireline_command(packet, &r, &i, &c, &p);
							snprintf(_debug_buf, sizeof(_debug_buf),
								"[SEC+2 RX] roll=%lu id=%lX cmd=%X payload=%lX",
								(unsigned long)r, (unsigned long)i, c, (unsigned long)p);
							_debug_print(_debug_buf);
						}
						process_packet(packet);
					}
				}
			}

			void reset_state() {
				this->state = {}; // clear all fields
				this->state.door_state = SecPlusCommon::DoorStatus::UNKNOWN;
				this->next_sync_time = 0;
			}

			bool detect() {
				this->begin();
				state_callback_t cb_backup = this->state_callback;
				this->state_callback = NULL; // temporarily disable callback since we are only doing detection
				this->reset_state();

				GARAGELIB_PRINTLN(F("Verifying SecPlus 2.0..."));
				// Also clear timers and buffers for a clean test run
				this->next_command_time = 0;
				while(buf.get_size() > 0) buf.pop();

				uint32_t startTime = millis();
				// Run for up to 5 seconds
				while (millis() - startTime < 5000) {

					// Your proposal Step 3: Call loop() repeatedly
					this->loop();

					// Your proposal Step 4: Check if the door status is no longer unknown
					if (this->state.door_state != SecPlusCommon::DoorStatus::UNKNOWN) {
						GARAGELIB_PRINTLN(F("Verification PASSED: State updated via internal loop."));
						//SecPlusCommon::sw_serial->end(); // Release the serial port
						this->state_callback = cb_backup; // recover original callback function
						return true;
					}
					yield();
				}

				GARAGELIB_PRINTLN(F("Verification FAILED: No valid status after 5s."));
				//SecPlusCommon::sw_serial->end(); // Release the serial port
				this->state_callback = cb_backup; // recover original callback function
				return false;
			}

			bool get_light_state() {
				return state.light_state;
			}

			bool get_lock_state() {
				return state.lock_state;
			}

			SecPlusCommon::DoorStatus get_door_state() {
				return state.door_state;
			}

			bool get_obstruction_state() {
				return state.obstruction_state;
			}

			uint16_t get_opening_count() {
				return state.openings;
			}

		private:
			uint32_t client_id;
			// Rolling code can just start at 0 since it will end up resycing while it tries to update it's state at the start.
			uint32_t rolling_code = 0;
			bool check_collision;
			bool ask_for_status = false;

			state_struct_t state {
				door_state: SecPlusCommon::DoorStatus::UNKNOWN,
				light_state: false,
				lock_state: false,
				obstruction_state: false,
				motor_state: false,
				button_state: false,
				battery_state: false,
				learn_state: false,
				openings: 0,
			};

			int rx_pin;
			int tx_pin;
			SecPlusReader reader;
			NEW_COMMAND_BUFFER(internal_buffer, SEC2_PACKET_SIZE, COMMAND_BUFFER_CAPACITY);
			SecPlusCommon::CommandBuffer buf = SecPlusCommon::CommandBuffer(SEC2_PACKET_SIZE, COMMAND_BUFFER_CAPACITY, internal_buffer);
			uint32_t next_command_time;
			uint32_t next_sync_time;

			state_callback_t state_callback = NULL;


			void update_callback() {
				if (state_callback) state_callback(state);
			}

			int8_t queue_command(Command command, uint8_t data_low, uint16_t data_high, uint16_t delay) {
				uint8_t *packet = buf.push_next();
				if (packet) {
					*packet = delay >> 8;
					*(packet + 1) = delay & 0xFF;
					return encode_data(static_cast<uint16_t>(command), data_low, data_high, packet + SecPlusCommon::metadata_size);
				} else {
					return -1;
				}
			}

			int8_t send_data(uint8_t *packet) {
				if(!SecPlusCommon::sw_serial) return -1;
				// Use mosfet to pull bus low (open drain) to take control of bus
				digitalWrite(tx_pin, HIGH);
				delayMicroseconds(1300);
				digitalWrite(tx_pin, LOW);
				delayMicroseconds(130);

				// Check if there is a bus collision
				if (check_collision && digitalRead(rx_pin)) return -1;


				if (GARAGELIB_DEBUG_ENABLED) {
					uint32_t r = 0;
					uint64_t i = 0;
					uint16_t c = 0;
					uint32_t p = 0;
					decode_wireline_command(packet, &r, &i, &c, &p);
					snprintf(_debug_buf, sizeof(_debug_buf),
						"[SEC+2 TX] roll=%lu id=%lX cmd=%X payload=%lX",
						(unsigned long)r, (unsigned long)i, c, (unsigned long)p);
					_debug_print(_debug_buf);
				}

				SecPlusCommon::sw_serial->write(packet, SEC2_PACKET_SIZE);
				delayMicroseconds(100);

				return 0;
			}

			int8_t encode_data(uint16_t command, uint8_t data_low, uint16_t data_high, uint8_t* packet) {
				// Pairty from ratgdo seems to be unused so data is encoded without it
				uint64_t fixed = client_id | (((uint64_t) command & 0xF00) << 24);
				uint32_t data = (command & 0xFF) | (((uint64_t) data_low & 0xF) << 8) | ((uint64_t) data_high << 16);
				int8_t res = encode_wireline(rolling_code, fixed, data, packet);
				rolling_code = (rolling_code + 1) & 0xfffffff;
				return res;
			}

			int8_t process_packet(uint8_t* packet) {
				uint32_t rolling;
				uint64_t fixed;
				uint32_t data;
				int8_t err = decode_wireline(packet, &rolling, &fixed, &data);
				if (err < 0) return err;

				Command command = static_cast<Command>(((fixed >> 24) & 0xf00) | (data & 0xff));

				switch (command) {
					case Command::STATUS:
						state.door_state = static_cast<SecPlusCommon::DoorStatus>((data >> 8) & 0xF);
						state.obstruction_state = !((data >> 22) & 0x1); // invert the logic as this seems to be active low
						state.lock_state = (data >> 24) & 0x1;
						state.light_state = (data >> 25) & 0x1;
						update_callback();
						break;
					case Command::LOCK:
						switch ((data >> 8) & 0b11) {
							case 0b00:
								state.lock_state = false;
							case 0b01:
								state.lock_state = true;
							case 0b10:
								state.lock_state = !state.lock_state;
								break;
							case 0b11:
								// Unknown
								break;
						}
						update_callback();
						break;
					case Command::LIGHT:
						switch ((data >> 8) & 0b11) {
							case 0b00:
								state.light_state = false;
							case 0b01:
								state.light_state = true;
							case 0b10:
								state.light_state = !state.light_state;
								break;
							case 0b11:
								// Unknown
								break;
						}
						update_callback();
						break;
					case Command::OPENINGS:
						// Openings is stored in the higher 2 bytes as a little endian unsigned short
						state.openings = ((data >> 8) & 0xFF00) | ((data >> 24) & 0xFF);
						update_callback();
						break;
					case Command::PAIR_3_RESP:
					case Command::OBST_1:
						ask_for_status = true; // possible obstruction detected, request status asap
						break;
					default:
						// Unknown command
						break;
				}

				return 0;
			}
	};
}

namespace SecPlus1 {
	// From gdolib
	// Send packet each 250 ms
	const uint8_t wall_panel_commands[] = { 0x35, 0x35, 0x35, 0x35, 0x33, 0x33, 0x53, 0x53, 0x38,
									0x3A, 0x3A, 0x3A, 0x39, 0x38, 0x3A, 0x38, 0x3A, 0x39, 0x3A };

	enum class Command : uint8_t
	{
		DOOR_BUTTON_PRESS = 0x30,
		DOOR_BUTTON_RELEASE = 0x31,
		LIGHT_BUTTON_PRESS = 0x32,
		LIGHT_BUTTON_RELEASE = 0x33,
		LOCK_BUTTON_PRESS = 0x34,
		LOCK_BUTTON_RELEASE = 0x35,

		UNKOWN_0X36 = 0x36,
		UNKNOWN_0X37 = 0x37,

		DOOR_STATUS = 0x38,
		OBSTRUCTION_STATUS = 0x39, // this is not proven
		LIGHT_LOCK_STATUS = 0x3A,
		UNKNOWN = 0xFF
	};

	enum class PanelEmuStatus : uint8_t
	{
		INACTIVE = 0,
		ACTIVE = 1,
		DETECTING = 2
	};

	typedef struct {
		SecPlusCommon::DoorStatus door_state;
		bool light_state;
		bool lock_state;
		bool obstruction_state;
	} state_struct_t;

	typedef void (*state_callback_t)(state_struct_t state);

	const size_t COMMAND_BUFFER_CAPACITY = 20;

	class Garage {
		public:
			Garage(int rx_pin, int tx_pin, bool check_collision = true) {
				this->check_collision = check_collision;
				this->rx_pin = rx_pin;
				this->tx_pin = tx_pin;
			}

			void begin() {
				SecPlusCommon::delete_sw_serial();
				SecPlusCommon::sw_serial = new SoftwareSerial();
				// Start Serial with 8 bits even parity parity 1 stop bit and inverted
				SecPlusCommon::sw_serial->begin(1200, SWSERIAL_8E1, rx_pin, tx_pin, true);
				this->panel_detection_start = millis();
			}

			void enable_callback(state_callback_t callback) {
				this->state_callback = callback;
			}

			int8_t request_door_status() {
				return queue_command(Command::DOOR_STATUS, 100);
			}

			int8_t request_obstruction_status() {
				return queue_command(Command::OBSTRUCTION_STATUS, 100);
			}

			int8_t request_light_lock_status() {
				return queue_command(Command::LIGHT_LOCK_STATUS, 100);
			}

			int8_t toggle_lock() {
				queue_command(Command::LOCK_BUTTON_PRESS, 250);
				queue_command(Command::LOCK_BUTTON_RELEASE, 40);
				return queue_command(Command::LOCK_BUTTON_RELEASE, 40);
			}

			int8_t toggle_light() {
				queue_command(Command::LIGHT_BUTTON_PRESS, 250);
				queue_command(Command::LIGHT_BUTTON_RELEASE, 40);
				return queue_command(Command::LIGHT_BUTTON_RELEASE, 40);
			}

			int8_t toggle_door() {
				queue_command(Command::DOOR_BUTTON_PRESS, 250);
				queue_command(Command::DOOR_BUTTON_RELEASE, 40);
				return queue_command(Command::DOOR_BUTTON_RELEASE, 40);
			}

			void loop() {
				if(!SecPlusCommon::sw_serial) return;

				if (!SecPlusCommon::sw_serial->available()) {
					switch (this->panel_emu_status) {
						case PanelEmuStatus::DETECTING:
							// Check for success: Did process_packet() update our door state?
							if (state.door_state != SecPlusCommon::DoorStatus::UNKNOWN) {
								GARAGELIB_PRINTLN(F("Existing wall panel detected. Panel Emu will remain inactive."));
								panel_emu_status = PanelEmuStatus::INACTIVE;
								emulation_command_index = 0;
							}
							// Check for timeout: 20 seconds have passed without finding a panel
							else if ((long)(millis() - panel_detection_start) > 20000) {
								GARAGELIB_PRINTLN(F("No wall panel detected after 20s. Starting active emulation."));
								panel_emu_status = PanelEmuStatus::ACTIVE; // Transition to Active (Emulating)
								emulation_command_index = 0;
							}
							break;

						case PanelEmuStatus::ACTIVE:
							// This is the active wall panel emulation from ratgdo/gdo.c
							if ((long)(millis() - next_sync_time) > 0) {
								uint8_t command_to_send = wall_panel_commands[emulation_command_index];
								send_data(&command_to_send);
								emulation_command_index++;
								if (emulation_command_index == 18) {
									emulation_command_index = 15;
								}
								next_sync_time = millis() + 250;
							}
							break;

						default:
							// Do nothing
							break;
					}

					if (buf.get_size() && (long)(millis() - next_command_time) > 0) {
						send_data(buf.get_head());
						buf.pop();
						next_command_time = millis() + buf.get_delay();
					}
				} else {
					process_packet(SecPlusCommon::sw_serial->read());
				}
			}

			void reset_state() {
				this->state = {}; // clear all fields
				this->state.door_state = SecPlusCommon::DoorStatus::UNKNOWN;
				this->current_command = 0;
				this->panel_emu_status = PanelEmuStatus::DETECTING;
				this->panel_detection_start = millis();
			}

			bool detect() {
				this->begin();
				state_callback_t cb_backup = this->state_callback;
				this->state_callback = NULL; // temporarily disable callback since we are only doing detection
				this->reset_state();

				// --- PHASE 1: Passive Listening (2.5 seconds) ---
				// Listen for an existing wall panel, which is usually very chatty.
				GARAGELIB_PRINTLN(F("Verifying Sec+ 1.0 -- Phase 1: Passively listening for existing panel..."));
				unsigned long phase1_startTime = millis();
				while (millis() - phase1_startTime < 2500) {
					if (SecPlusCommon::sw_serial->available()) {
						process_packet(SecPlusCommon::sw_serial->read());
						// Check if we received and decoded a valid status packet
						if (this->state.door_state != SecPlusCommon::DoorStatus::UNKNOWN) {
							GARAGELIB_PRINTLN(F("Verification PASSED (Passive): Existing Sec+ 1.0 panel detected."));
							//SecPlusCommon::sw_serial->end(); // Release the serial port
							this->state_callback = cb_backup; // recover original callback function
							return true;
						}
					}
					yield();
				}

				// --- PHASE 2: Active Probing (5 seconds) ---
				// If Phase 1 failed, actively emulate a wall panel to get a response from the opener.
				GARAGELIB_PRINTLN(F("Verifying Sec+ 1.0 -- Phase 2: No panel detected, actively probing opener..."));
				unsigned long phase2_startTime = millis();
				uint32_t last_emulation_time = 0;
				uint8_t emulation_command_index = 0;

				while (millis() - phase2_startTime < 5000) {
					if (SecPlusCommon::sw_serial->available()) {
						process_packet(SecPlusCommon::sw_serial->read());
						if (this->state.door_state != SecPlusCommon::DoorStatus::UNKNOWN) {
							GARAGELIB_PRINTLN(F("Verification PASSED (Active): Opener responded to probe."));
							//SecPlusCommon::sw_serial->end(); // Release the serial port
							this->state_callback = cb_backup;
							return true;
						}
					} else { // Emulation Sender: Send a command every 250ms
						if (millis() - last_emulation_time >= 250) {
							last_emulation_time = millis();
							uint8_t command_to_send = wall_panel_commands[emulation_command_index];
							send_data(&command_to_send);
							emulation_command_index++;
							if (emulation_command_index == 18) {
								emulation_command_index = 15;
							}
						}
					}
					yield();
				}

				GARAGELIB_PRINTLN(F("Verification FAILED: No Sec+ 1.0 communication detected."));
				//SecPlusCommon::sw_serial->end(); // Release the serial port
				this->state_callback = cb_backup;
				return false;
			}

			bool get_light_state() {
				return state.light_state;
			}

			bool get_lock_state() {
				return state.lock_state;
			}

			SecPlusCommon::DoorStatus get_door_state() {
				return state.door_state;
			}

			bool get_obstruction_state() {
				return state.obstruction_state;
			}

			uint8_t get_panel_emu_status() {
				return static_cast<uint8_t>(panel_emu_status);
			}

		private:
			uint32_t client_id;
			bool check_collision;
			PanelEmuStatus panel_emu_status = PanelEmuStatus::DETECTING;
			uint32_t panel_detection_start = 0;

			state_struct_t state {
				door_state: SecPlusCommon::DoorStatus::UNKNOWN,
				light_state: false,
				lock_state: false,
				obstruction_state: false,
			};

			int rx_pin;
			int tx_pin;

			uint8_t current_command = 0;

			NEW_COMMAND_BUFFER(internal_buffer, 1, COMMAND_BUFFER_CAPACITY);
			SecPlusCommon::CommandBuffer buf = SecPlusCommon::CommandBuffer(1, COMMAND_BUFFER_CAPACITY, internal_buffer);
			uint32_t next_command_time;
			uint32_t next_sync_time;
			uint8_t emulation_command_index = 0;
			state_callback_t state_callback = NULL;


			void update_callback() {
				if (state_callback) state_callback(state);
			}

			int8_t queue_command(Command command, uint16_t delay) {
				uint8_t *packet = buf.push_next();
				if (packet) {
					*packet = delay >> 8;
					*(packet + 1) = delay & 0xFF;
					*(packet + SecPlusCommon::metadata_size) = static_cast<uint8_t>(command);
					return 0;
				} else {
					return -1;
				}
			}

			void send_data(uint8_t *packet) {
				if(!SecPlusCommon::sw_serial) return;
				if (GARAGELIB_DEBUG_ENABLED) {
					snprintf(_debug_buf, sizeof(_debug_buf), "[SEC+1 TX] cmd=%02X", *packet);
					_debug_print(_debug_buf);
				}

				SecPlusCommon::sw_serial->write(packet, 1);
				delayMicroseconds(100);
			}

			int8_t process_packet(uint8_t packet) {
				if (!current_command) {
					Command command = static_cast<Command>(packet);
					switch (command) {
						case Command::DOOR_STATUS:
						case Command::OBSTRUCTION_STATUS:
						case Command::LIGHT_LOCK_STATUS:
							// Requires second byte
							current_command = packet;
							break;
						default:
							// Only handle the status commands for now
							if (GARAGELIB_DEBUG_ENABLED) {
								snprintf(_debug_buf, sizeof(_debug_buf), "[SEC+1 RX] cmd=%02X", packet);
								_debug_print(_debug_buf);
							}
							break;
					}
				} else {
					if (GARAGELIB_DEBUG_ENABLED) {
						snprintf(_debug_buf, sizeof(_debug_buf), "[SEC+1 RX] cmd=%02X data=%02X", current_command, packet);
						_debug_print(_debug_buf);
					}
					Command command = static_cast<Command>(current_command);
					switch (command) {
						case Command::DOOR_STATUS: {
							uint8_t val = packet & 0x7;
							switch (val) {
								case 0x00:
								case 0x06:
									state.door_state = SecPlusCommon::DoorStatus::STOPPED;
									break;
								case 0x01:
									state.door_state = SecPlusCommon::DoorStatus::OPENING;
									break;
								case 0x02:
									state.door_state = SecPlusCommon::DoorStatus::OPEN;
									break;
								// No 0x03
								case 0x04:
									state.door_state = SecPlusCommon::DoorStatus::CLOSING;
									break;
								case 0x05:
									state.door_state = SecPlusCommon::DoorStatus::CLOSED;
									break;
								default:
									state.door_state = SecPlusCommon::DoorStatus::UNKNOWN;
							}
							update_callback();
							break;
						}
						case Command::OBSTRUCTION_STATUS:
							state.obstruction_state = packet != 0;
							update_callback();
							break;
						case Command::LIGHT_LOCK_STATUS:
							// upper nibble must be 5
							if ((packet & 0xF0) != 0x50) {
								break;
							}

							// Light state in bit 2
							state.light_state = (packet >> 2) & 1;
							// Light state in bit 3, but inverted
							state.lock_state = (~packet >> 3) & 1;
							update_callback();
							break;
						default:
							// Unreachable
							break;
					}
					current_command = 0;
				}
				return 0;
			}
	};
}