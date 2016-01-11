// MESSAGE PIXY_CAM_PTS PACKING

#define MAVLINK_MSG_ID_PIXY_CAM_PTS 180

typedef struct __mavlink_pixy_cam_pts_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint64_t count; /*< Number of valid points*/
 float x[4]; /*< X-coordinate of detected point*/
 float y[4]; /*< Y-coordinate of detected point*/
 float width[4]; /*< Width of bounding box*/
 float height[4]; /*< Height of bounding box*/
} mavlink_pixy_cam_pts_t;

#define MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN 80
#define MAVLINK_MSG_ID_180_LEN 80

#define MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC 222
#define MAVLINK_MSG_ID_180_CRC 222

#define MAVLINK_MSG_PIXY_CAM_PTS_FIELD_X_LEN 4
#define MAVLINK_MSG_PIXY_CAM_PTS_FIELD_Y_LEN 4
#define MAVLINK_MSG_PIXY_CAM_PTS_FIELD_WIDTH_LEN 4
#define MAVLINK_MSG_PIXY_CAM_PTS_FIELD_HEIGHT_LEN 4

#define MAVLINK_MESSAGE_INFO_PIXY_CAM_PTS { \
	"PIXY_CAM_PTS", \
	6, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pixy_cam_pts_t, time_usec) }, \
         { "count", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_pixy_cam_pts_t, count) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 4, 16, offsetof(mavlink_pixy_cam_pts_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 4, 32, offsetof(mavlink_pixy_cam_pts_t, y) }, \
         { "width", NULL, MAVLINK_TYPE_FLOAT, 4, 48, offsetof(mavlink_pixy_cam_pts_t, width) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 4, 64, offsetof(mavlink_pixy_cam_pts_t, height) }, \
         } \
}


/**
 * @brief Pack a pixy_cam_pts message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param count Number of valid points
 * @param x X-coordinate of detected point
 * @param y Y-coordinate of detected point
 * @param width Width of bounding box
 * @param height Height of bounding box
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint64_t count, const float *x, const float *y, const float *width, const float *height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf, 8, count);
	_mav_put_float_array(buf, 16, x, 4);
	_mav_put_float_array(buf, 32, y, 4);
	_mav_put_float_array(buf, 48, width, 4);
	_mav_put_float_array(buf, 64, height, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#else
	mavlink_pixy_cam_pts_t packet;
	packet.time_usec = time_usec;
	packet.count = count;
	mav_array_memcpy(packet.x, x, sizeof(float)*4);
	mav_array_memcpy(packet.y, y, sizeof(float)*4);
	mav_array_memcpy(packet.width, width, sizeof(float)*4);
	mav_array_memcpy(packet.height, height, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXY_CAM_PTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
}

/**
 * @brief Pack a pixy_cam_pts message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param count Number of valid points
 * @param x X-coordinate of detected point
 * @param y Y-coordinate of detected point
 * @param width Width of bounding box
 * @param height Height of bounding box
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint64_t count,const float *x,const float *y,const float *width,const float *height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf, 8, count);
	_mav_put_float_array(buf, 16, x, 4);
	_mav_put_float_array(buf, 32, y, 4);
	_mav_put_float_array(buf, 48, width, 4);
	_mav_put_float_array(buf, 64, height, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#else
	mavlink_pixy_cam_pts_t packet;
	packet.time_usec = time_usec;
	packet.count = count;
	mav_array_memcpy(packet.x, x, sizeof(float)*4);
	mav_array_memcpy(packet.y, y, sizeof(float)*4);
	mav_array_memcpy(packet.width, width, sizeof(float)*4);
	mav_array_memcpy(packet.height, height, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXY_CAM_PTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
}

/**
 * @brief Encode a pixy_cam_pts struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pixy_cam_pts C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pixy_cam_pts_t* pixy_cam_pts)
{
	return mavlink_msg_pixy_cam_pts_pack(system_id, component_id, msg, pixy_cam_pts->time_usec, pixy_cam_pts->count, pixy_cam_pts->x, pixy_cam_pts->y, pixy_cam_pts->width, pixy_cam_pts->height);
}

/**
 * @brief Encode a pixy_cam_pts struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pixy_cam_pts C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pixy_cam_pts_t* pixy_cam_pts)
{
	return mavlink_msg_pixy_cam_pts_pack_chan(system_id, component_id, chan, msg, pixy_cam_pts->time_usec, pixy_cam_pts->count, pixy_cam_pts->x, pixy_cam_pts->y, pixy_cam_pts->width, pixy_cam_pts->height);
}

/**
 * @brief Send a pixy_cam_pts message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param count Number of valid points
 * @param x X-coordinate of detected point
 * @param y Y-coordinate of detected point
 * @param width Width of bounding box
 * @param height Height of bounding box
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pixy_cam_pts_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t count, const float *x, const float *y, const float *width, const float *height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf, 8, count);
	_mav_put_float_array(buf, 16, x, 4);
	_mav_put_float_array(buf, 32, y, 4);
	_mav_put_float_array(buf, 48, width, 4);
	_mav_put_float_array(buf, 64, height, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
#else
	mavlink_pixy_cam_pts_t packet;
	packet.time_usec = time_usec;
	packet.count = count;
	mav_array_memcpy(packet.x, x, sizeof(float)*4);
	mav_array_memcpy(packet.y, y, sizeof(float)*4);
	mav_array_memcpy(packet.width, width, sizeof(float)*4);
	mav_array_memcpy(packet.height, height, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, (const char *)&packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, (const char *)&packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pixy_cam_pts_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t count, const float *x, const float *y, const float *width, const float *height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf, 8, count);
	_mav_put_float_array(buf, 16, x, 4);
	_mav_put_float_array(buf, 32, y, 4);
	_mav_put_float_array(buf, 48, width, 4);
	_mav_put_float_array(buf, 64, height, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, buf, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
#else
	mavlink_pixy_cam_pts_t *packet = (mavlink_pixy_cam_pts_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->count = count;
	mav_array_memcpy(packet->x, x, sizeof(float)*4);
	mav_array_memcpy(packet->y, y, sizeof(float)*4);
	mav_array_memcpy(packet->width, width, sizeof(float)*4);
	mav_array_memcpy(packet->height, height, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, (const char *)packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN, MAVLINK_MSG_ID_PIXY_CAM_PTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXY_CAM_PTS, (const char *)packet, MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PIXY_CAM_PTS UNPACKING


/**
 * @brief Get field time_usec from pixy_cam_pts message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_pixy_cam_pts_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field count from pixy_cam_pts message
 *
 * @return Number of valid points
 */
static inline uint64_t mavlink_msg_pixy_cam_pts_get_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field x from pixy_cam_pts message
 *
 * @return X-coordinate of detected point
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_get_x(const mavlink_message_t* msg, float *x)
{
	return _MAV_RETURN_float_array(msg, x, 4,  16);
}

/**
 * @brief Get field y from pixy_cam_pts message
 *
 * @return Y-coordinate of detected point
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_get_y(const mavlink_message_t* msg, float *y)
{
	return _MAV_RETURN_float_array(msg, y, 4,  32);
}

/**
 * @brief Get field width from pixy_cam_pts message
 *
 * @return Width of bounding box
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_get_width(const mavlink_message_t* msg, float *width)
{
	return _MAV_RETURN_float_array(msg, width, 4,  48);
}

/**
 * @brief Get field height from pixy_cam_pts message
 *
 * @return Height of bounding box
 */
static inline uint16_t mavlink_msg_pixy_cam_pts_get_height(const mavlink_message_t* msg, float *height)
{
	return _MAV_RETURN_float_array(msg, height, 4,  64);
}

/**
 * @brief Decode a pixy_cam_pts message into a struct
 *
 * @param msg The message to decode
 * @param pixy_cam_pts C-struct to decode the message contents into
 */
static inline void mavlink_msg_pixy_cam_pts_decode(const mavlink_message_t* msg, mavlink_pixy_cam_pts_t* pixy_cam_pts)
{
#if MAVLINK_NEED_BYTE_SWAP
	pixy_cam_pts->time_usec = mavlink_msg_pixy_cam_pts_get_time_usec(msg);
	pixy_cam_pts->count = mavlink_msg_pixy_cam_pts_get_count(msg);
	mavlink_msg_pixy_cam_pts_get_x(msg, pixy_cam_pts->x);
	mavlink_msg_pixy_cam_pts_get_y(msg, pixy_cam_pts->y);
	mavlink_msg_pixy_cam_pts_get_width(msg, pixy_cam_pts->width);
	mavlink_msg_pixy_cam_pts_get_height(msg, pixy_cam_pts->height);
#else
	memcpy(pixy_cam_pts, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PIXY_CAM_PTS_LEN);
#endif
}
