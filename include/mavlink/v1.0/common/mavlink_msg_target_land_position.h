// MESSAGE TARGET_LAND_POSITION PACKING

#define MAVLINK_MSG_ID_TARGET_LAND_POSITION 161

typedef struct __mavlink_target_land_position_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84, in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
 float x; /*< x offset to target in vehicle NED frame in m*/
 float y; /*< y offset to target in vehicle NED frame in m*/
 float z; /*< z offset to target in vehicle NED frame in m*/
 float yaw; /*< yaw offset to target in vehicle NED frame in radians*/
} mavlink_target_land_position_t;

#define MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN 36
#define MAVLINK_MSG_ID_161_LEN 36

#define MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC 68
#define MAVLINK_MSG_ID_161_CRC 68



#define MAVLINK_MESSAGE_INFO_TARGET_LAND_POSITION { \
	"TARGET_LAND_POSITION", \
	8, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_target_land_position_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_target_land_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_target_land_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_target_land_position_t, alt) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_land_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_land_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_target_land_position_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_target_land_position_t, yaw) }, \
         } \
}


/**
 * @brief Pack a target_land_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84, in degrees * 1E7
 * @param alt Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x x offset to target in vehicle NED frame in m
 * @param y y offset to target in vehicle NED frame in m
 * @param z z offset to target in vehicle NED frame in m
 * @param yaw yaw offset to target in vehicle NED frame in radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_land_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_float(buf, 20, x);
	_mav_put_float(buf, 24, y);
	_mav_put_float(buf, 28, z);
	_mav_put_float(buf, 32, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#else
	mavlink_target_land_position_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_LAND_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
}

/**
 * @brief Pack a target_land_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84, in degrees * 1E7
 * @param alt Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x x offset to target in vehicle NED frame in m
 * @param y y offset to target in vehicle NED frame in m
 * @param z z offset to target in vehicle NED frame in m
 * @param yaw yaw offset to target in vehicle NED frame in radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_land_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,int32_t lat,int32_t lon,int32_t alt,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_float(buf, 20, x);
	_mav_put_float(buf, 24, y);
	_mav_put_float(buf, 28, z);
	_mav_put_float(buf, 32, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#else
	mavlink_target_land_position_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TARGET_LAND_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
}

/**
 * @brief Encode a target_land_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_land_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_land_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_land_position_t* target_land_position)
{
	return mavlink_msg_target_land_position_pack(system_id, component_id, msg, target_land_position->time_usec, target_land_position->lat, target_land_position->lon, target_land_position->alt, target_land_position->x, target_land_position->y, target_land_position->z, target_land_position->yaw);
}

/**
 * @brief Encode a target_land_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_land_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_land_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_land_position_t* target_land_position)
{
	return mavlink_msg_target_land_position_pack_chan(system_id, component_id, chan, msg, target_land_position->time_usec, target_land_position->lat, target_land_position->lon, target_land_position->alt, target_land_position->x, target_land_position->y, target_land_position->z, target_land_position->yaw);
}

/**
 * @brief Send a target_land_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84, in degrees * 1E7
 * @param alt Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x x offset to target in vehicle NED frame in m
 * @param y y offset to target in vehicle NED frame in m
 * @param z z offset to target in vehicle NED frame in m
 * @param yaw yaw offset to target in vehicle NED frame in radians
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_land_position_send(mavlink_channel_t chan, uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_float(buf, 20, x);
	_mav_put_float(buf, 24, y);
	_mav_put_float(buf, 28, z);
	_mav_put_float(buf, 32, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
#else
	mavlink_target_land_position_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, (const char *)&packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, (const char *)&packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_land_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_float(buf, 20, x);
	_mav_put_float(buf, 24, y);
	_mav_put_float(buf, 28, z);
	_mav_put_float(buf, 32, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, buf, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
#else
	mavlink_target_land_position_t *packet = (mavlink_target_land_position_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, (const char *)packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN, MAVLINK_MSG_ID_TARGET_LAND_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_LAND_POSITION, (const char *)packet, MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TARGET_LAND_POSITION UNPACKING


/**
 * @brief Get field time_usec from target_land_position message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_target_land_position_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from target_land_position message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_target_land_position_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from target_land_position message
 *
 * @return Longitude (WGS84, in degrees * 1E7
 */
static inline int32_t mavlink_msg_target_land_position_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from target_land_position message
 *
 * @return Altitude (AMSL), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_target_land_position_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field x from target_land_position message
 *
 * @return x offset to target in vehicle NED frame in m
 */
static inline float mavlink_msg_target_land_position_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field y from target_land_position message
 *
 * @return y offset to target in vehicle NED frame in m
 */
static inline float mavlink_msg_target_land_position_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field z from target_land_position message
 *
 * @return z offset to target in vehicle NED frame in m
 */
static inline float mavlink_msg_target_land_position_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw from target_land_position message
 *
 * @return yaw offset to target in vehicle NED frame in radians
 */
static inline float mavlink_msg_target_land_position_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a target_land_position message into a struct
 *
 * @param msg The message to decode
 * @param target_land_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_land_position_decode(const mavlink_message_t* msg, mavlink_target_land_position_t* target_land_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	target_land_position->time_usec = mavlink_msg_target_land_position_get_time_usec(msg);
	target_land_position->lat = mavlink_msg_target_land_position_get_lat(msg);
	target_land_position->lon = mavlink_msg_target_land_position_get_lon(msg);
	target_land_position->alt = mavlink_msg_target_land_position_get_alt(msg);
	target_land_position->x = mavlink_msg_target_land_position_get_x(msg);
	target_land_position->y = mavlink_msg_target_land_position_get_y(msg);
	target_land_position->z = mavlink_msg_target_land_position_get_z(msg);
	target_land_position->yaw = mavlink_msg_target_land_position_get_yaw(msg);
#else
	memcpy(target_land_position, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TARGET_LAND_POSITION_LEN);
#endif
}
