/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/#ifndef RECEIVERCANMESSAGEDEF_H
#define RECEIVERCANMESSAGEDEF_H

#include "SensorCoreClassesGlobal.h"

// len
#define RECEIVERCANMSG_LEN 8

// CAN Frame
typedef struct {
    uint32_t id;        // Message id
    uint32_t timestamp; // timestamp in milliseconds
    uint8_t  flags;     // [extended_id|1][RTR:1][reserver:6]
    uint8_t  len;       // Frame size (0.8)
    uint8_t  data[RECEIVERCANMSG_LEN];   // Databytes 0..7
    uint8_t  pad1;      // Padding required so that sizeof(ReceiverCANMessage) be 20 on all platforms
    uint8_t  pad2;      // ...
} ReceiverCANMessage;


// id
#define RECEIVERCANMSG_ID_SENSORSTATUS                   1
#define RECEIVERCANMSG_ID_SENSORBOOT                     2
#define RECEIVERCANMSG_ID_COMPLETEDFRAME                 9
#define RECEIVERCANMSG_ID_OBSTACLETRACK                  10
#define RECEIVERCANMSG_ID_OBSTACLEVELOCITY               11
#define RECEIVERCANMSG_ID_OBSTACLESIZE                   12
#define RECEIVERCANMSG_ID_OBSTACLEANGULARPOSITION        13
#define RECEIVERCANMSG_ID_CHANNELDISTANCE1_FIRST         20
#define RECEIVERCANMSG_ID_CHANNELDISTANCE1_LAST          26
#define RECEIVERCANMSG_ID_CHANNELDISTANCE2_FIRST         30
#define RECEIVERCANMSG_ID_CHANNELDISTANCE2_LAST          36
#define RECEIVERCANMSG_ID_CHANNELINTENSITY1_FIRST        40
#define RECEIVERCANMSG_ID_CHANNELINTENSITY1_LAST         46
#define RECEIVERCANMSG_ID_CHANNELINTENSITY2_FIRST        50
#define RECEIVERCANMSG_ID_CHANNELINTENSITY2_LAST         56
#define RECEIVERCANMSG_ID_CHANNELDISTANCEANDINTENSITY    60
#define RECEIVERCANMSG_ID_COMMANDMESSAGE                 80


// New command id for Wagner/Guardian
#define RECEIVERCANMSG_ID_POLL                           86
#define RECEIVERCANMSG_ID_GETDATA                        87
#define RECEIVERCANMSG_ID_POLLMESSAGES                   88
#define RECEIVERCANMSG_ID_LIDARQUERY                     89

/*
00: Command (0xC0 = SET_PARAMETER)
0xC1 = QUERY_PARAMETER)
0xC2 = RESPONSE_PARAMETER)
01: Type (0x01 = ALGO_SELECTED
0x02 = ALGO_PARAMETER
0x03 = AWL_REGISTER
0x04 = BIAS
0x05 = ADC_REGISTER
0x06 = PRESET
0x07 = GLOBAL_PARAMETER
0x08 = GPIO_CONTROL
0x11 = TRACKER_SELECTED
0x12 = TRACKER_PARAMETER
0x20 = DATE_TIME
0xD0 = RECORD_FILENAME (zero-terminated)
0xD1 = PLAYBACK_FILENAME (zero-terminated)
02-03: Address (U16_LE)
04-07: Value (x32_LE or U8S)

for DATE:
04-05: YEAR (U16_LE)
06: MONTH
07: DAY-OF-MONTH

for TIME:
04: HOURS
05: MINUTES
06: SECONDS
07: 0x00
*/

#define RECEIVERCANMSG_ID_CMD_SET_PARAMETER            0xC0
#define RECEIVERCANMSG_ID_CMD_QUERY_PARAMETER          0xC1
#define RECEIVERCANMSG_ID_CMD_RESPONSE_PARAMETER       0xC2

#define RECEIVERCANMSG_ID_CMD_PLAYBACK_RAW             0xD1
#define RECEIVERCANMSG_ID_CMD_RECORD_CALIBRATION       0xDA
#define RECEIVERCANMSG_ID_CMD_TRANSMIT_RAW             0xE0
#define RECEIVERCANMSG_ID_CMD_TRANSMIT_COOKED          0xE1

#define RECEIVERCANMSG_ID_CMD_PARAM_ALGO_SELECTED      0x01
#define RECEIVERCANMSG_ID_CMD_PARAM_ALGO_PARAMETER     0x02
#define RECEIVERCANMSG_ID_CMD_PARAM_AWL_REGISTER       0x03
#define RECEIVERCANMSG_ID_CMD_PARAM_BIAS               0x04
#define RECEIVERCANMSG_ID_CMD_PARAM_ADC_REGISTER       0x05
#define RECEIVERCANMSG_ID_CMD_PARAM_PRESET             0x06
#define RECEIVERCANMSG_ID_CMD_PARAM_GLOBAL_PARAMETER   0x07
#define RECEIVERCANMSG_ID_CMD_PARAM_GPIO_CONTROL       0x08
#define RECEIVERCANMSG_ID_CMD_PARAM_TRACKER_SELECTED   0x11
#define RECEIVERCANMSG_ID_CMD_PARAM_TRACKER_PARAMETER  0x12
#define RECEIVERCANMSG_ID_CMD_PARAM_DATE_TIME          0x20
#define RECEIVERCANMSG_ID_CMD_PARAM_RECORD_FILENAME    0xD0
#define RECEIVERCANMSG_ID_CMD_PARAM_PLAYBACK_FILENAME  0xD1

#define RECEIVERCANMSG_ID_CMD_PARAM_SENSOR_SPECIFIC    0x24
#define RECEIVERCANMSG_ID_CMD_SSP_ENABLE_SYSTEM	  0x10
#define RECEIVERCANMSG_ID_CMD_SSP_FRAME_RATE		  0x11
#define RECEIVERCANMSG_ID_CMD_SSP_ENABLE_LASER	  0x31
#define RECEIVERCANMSG_ID_CMD_SSP_ENABLE_AUTO_GAIN	  0x32
#define RECEIVERCANMSG_ID_CMD_SSP_ENABLE_DC_BALANCE	  0x33


#endif //RECEIVERCANMESSAGEDEF_H
