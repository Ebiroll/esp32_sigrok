
#ifndef PROTOCOL_H
#define PROTOCOL_H

//******************************************************************************

#define DISC_QUERY_PORT  (49393)
#define DISC_RESP_PORT  (49394)
#define DISC_BCAST_ADDR  ("225.0.0.50")


#define PKT_HEADER_SIZE  (8)

#define PKT_CMD(msg)  (((uint16_t)(msg)[0] << 8) | (msg)[1])
#define PKT_SetCMD(msg, cmd)  do {(msg)[0] = (cmd) >> 8; (msg)[1] = (cmd) & 0xFF;} while(0)

#define PKT_SEQ(msg)  ((msg)[2])
#define PKT_SEQ2(msg)  ((msg)[3])
#define PKT_SEQ_GOOD(msg)  ((msg)[2] == (0xFF & ~((msg)[3])))
#define PKT_SetSeq(msg, seq)  do {(msg)[2] = (seq); (msg)[3] = (~(seq)) & 0xFF;} while(0)

#define PKT_PAYLOAD_SIZE(msg)  (ntohl(*(uint32_t*)&(msg)[4]))
#define PKT_SetPayloadSize(msg, size)  *(uint32_t*)&(msg)[4] = htonl(size)

#define PKT_SIZE(msg)  (PKT_PAYLOAD_SIZE(msg) + PKT_HEADER_SIZE)

#define PKT_PAYLOAD(msg)  (&(msg)[PKT_HEADER_SIZE])


#define CMD_PING            (0x0000)
#define CMD_DISCONNECT      (0x0002)
// #define CMD_SERVNAME        (0x0100)
// #define CMD_LIST_DEVICES    (0x0101)
#define CMD_CONNECT_DEVICE  (0x0200)
#define CMD_DEVICE_COMM     (0x0F00)

//******************************************************************************
#endif // PROTOCOL_H
