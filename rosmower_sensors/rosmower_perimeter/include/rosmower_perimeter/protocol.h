#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#define START_FRAME 0xDCBA

typedef struct {
   uint16_t start;
   int16_t  cmd;
   uint16_t checksum;
} SerialCommand;

typedef struct {
   uint16_t start;
   int16_t  left_mag;
   int16_t  right_mag;
   int16_t  left_smag;
   int16_t  right_smag;
   bool left_inside;
   bool right_inside;
   bool left_timeout;
   bool right_timeout;
   bool calibrated;
   bool bumperLeft;
   bool bumperRight;
   unsigned char buttonCount;
   uint16_t checksum;
} SerialFeedback;

#endif
