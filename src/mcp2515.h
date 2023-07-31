#ifndef MCP2515_H
#define MCP2515_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "mcp2515_hal.h"

#define CAN_MSG_BUFFER_SIZE             10

#define SPI_TRANS_LEN                   32
#define CAN_STDID_MASK                  0x7FF //11bit

/*
Instruction
*/
#define MCP2515_INSTR_WRITE             0x02
#define MCP2515_INSTR_READ              0x03
#define MCP2515_INSTR_BITMOD            0x05
#define MCP2515_INSTR_LOAD_TX0          0x40
#define MCP2515_INSTR_LOAD_TX1          0x42
#define MCP2515_INSTR_LOAD_TX2          0x44
#define MCP2515_INSTR_RTS               0x80
#define MCP2515_INSTR_RTS_TX0           0x81
#define MCP2515_INSTR_RTS_TX1           0x82
#define MCP2515_INSTR_RTS_TX2           0x84
#define MCP2515_INSTR_RTS_ALL           0x87
#define MCP2515_INSTR_READ_RX0          0x90
#define MCP2515_INSTR_READ_RX1          0x94
#define MCP2515_INSTR_READ_STATUS       0xA0
#define MCP2515_INSTR_RX_STATUS         0xB0
#define MCP2515_INSTR_RESET             0xC0

/*
Registers
*/
#define MCP2515_REG_RXF0SIDH             0x00
#define MCP2515_REG_RXF0SIDL             0x01
#define MCP2515_REG_RXF0EID8             0x02
#define MCP2515_REG_RXF0EID0             0x03
#define MCP2515_REG_RXF1SIDH             0x04
#define MCP2515_REG_RXF1SIDL             0x05
#define MCP2515_REG_RXF1EID8             0x06
#define MCP2515_REG_RXF1EID0             0x07
#define MCP2515_REG_RXF2SIDH             0x08
#define MCP2515_REG_RXF2SIDL             0x09
#define MCP2515_REG_RXF2EID8             0x0A
#define MCP2515_REG_RXF2EID0             0x0B
#define MCP2515_REG_CANSTAT              0x0E
#define MCP2515_REG_CANCTRL              0x0F
#define MCP2515_REG_RXF3SIDH             0x10
#define MCP2515_REG_RXF3SIDL             0x11
#define MCP2515_REG_RXF3EID8             0x12
#define MCP2515_REG_RXF3EID0             0x13
#define MCP2515_REG_RXF4SIDH             0x14
#define MCP2515_REG_RXF4SIDL             0x15
#define MCP2515_REG_RXF4EID8             0x16
#define MCP2515_REG_RXF4EID0             0x17
#define MCP2515_REG_RXF5SIDH             0x18
#define MCP2515_REG_RXF5SIDL             0x19
#define MCP2515_REG_RXF5EID8             0x1A
#define MCP2515_REG_RXF5EID0             0x1B
#define MCP2515_REG_TEC                  0x1C
#define MCP2515_REG_REC                  0x1D
#define MCP2515_REG_RXM0SIDH             0x20
#define MCP2515_REG_RXM0SIDL             0x21
#define MCP2515_REG_RXM0EID8             0x22
#define MCP2515_REG_RXM0EID0             0x23
#define MCP2515_REG_RXM1SIDH             0x24
#define MCP2515_REG_RXM1SIDL             0x25
#define MCP2515_REG_RXM1EID8             0x26
#define MCP2515_REG_RXM1EID0             0x27
#define MCP2515_REG_CNF3                 0x28
#define MCP2515_REG_CNF2                 0x29
#define MCP2515_REG_CNF1                 0x2A
#define MCP2515_REG_CANINTE              0x2B
#define MCP2515_REG_CANINTF              0x2C
#define MCP2515_REG_EFLG                 0x2D
#define MCP2515_REG_TXB0CTRL             0x30
#define MCP2515_REG_TXB0SIDH             0x31
#define MCP2515_REG_TXB0SIDL             0x32
#define MCP2515_REG_TXB0EID8             0x33
#define MCP2515_REG_TXB0EID0             0x34
#define MCP2515_REG_TXB0DLC              0x35
#define MCP2515_REG_TXB0DATA             0x36
#define MCP2515_REG_TXB1CTRL             0x40
#define MCP2515_REG_TXB1SIDH             0x41
#define MCP2515_REG_TXB1SIDL             0x42
#define MCP2515_REG_TXB1EID8             0x43
#define MCP2515_REG_TXB1EID0             0x44
#define MCP2515_REG_TXB1DLC              0x45
#define MCP2515_REG_TXB1DATA             0x46
#define MCP2515_REG_TXB2CTRL             0x50
#define MCP2515_REG_TXB2SIDH             0x51
#define MCP2515_REG_TXB2SIDL             0x52
#define MCP2515_REG_TXB2EID8             0x53
#define MCP2515_REG_TXB2EID0             0x54
#define MCP2515_REG_TXB2DLC              0x55
#define MCP2515_REG_TXB2DATA             0x56
#define MCP2515_REG_RXB0CTRL             0x60
#define MCP2515_REG_RXB0SIDH             0x61
#define MCP2515_REG_RXB0SIDL             0x62
#define MCP2515_REG_RXB0EID8             0x63
#define MCP2515_REG_RXB0EID0             0x64
#define MCP2515_REG_RXB0DLC              0x65
#define MCP2515_REG_RXB0DATA             0x66
#define MCP2515_REG_RXB1CTRL             0x70
#define MCP2515_REG_RXB1SIDH             0x71
#define MCP2515_REG_RXB1SIDL             0x72
#define MCP2515_REG_RXB1EID8             0x73
#define MCP2515_REG_RXB1EID0             0x74
#define MCP2515_REG_RXB1DLC              0x75
#define MCP2515_REG_RXB1DATA             0x76

/*
clock
*/
#define MCP2515_8MHz_1000kBPS_CFG1           0x00
#define MCP2515_8MHz_1000kBPS_CFG2           0x80
#define MCP2515_8MHz_1000kBPS_CFG3           0x80

#define MCP2515_8MHz_500kBPS_CFG1            0x00
#define MCP2515_8MHz_500kBPS_CFG2            0x90
#define MCP2515_8MHz_500kBPS_CFG3            0x82

#define MCP2515_8MHz_250kBPS_CFG1            0x00
#define MCP2515_8MHz_250kBPS_CFG2            0xB1
#define MCP2515_8MHz_250kBPS_CFG3            0x85


#define MCP2515_TXBnCTRL_TXP            (1 << 1)|(1 << 0)
#define MCP2515_TXBnCTRL_TXREQ          (1 << 3)
#define MCP2515_TXBnCTRL_TXERR          (1 << 4)
#define MCP2515_TXBnCTRL_MLOA           (1 << 5)
#define MCP2515_TXBnCTRL_ABTF           (1 << 6)


#define MCP2515_CANINTE_RX0IE           (1 << 0)
#define MCP2515_CANINTE_RX1IE           (1 << 1)
#define MCP2515_CANINTE_TX0IE           (1 << 2)
#define MCP2515_CANINTE_TX1IE           (1 << 3)
#define MCP2515_CANINTE_TX2IE           (1 << 4)
#define MCP2515_CANINTE_ERRIE           (1 << 5)
#define MCP2515_CANINTE_WAKIE           (1 << 6)
#define MCP2515_CANINTE_MERRE           (1 << 7)


#define MCP2515_FASTRX_LEN              14 //dymmy byte + 4 byte ID + dlc + 8 data
#define MCP2515_FASTTX_LEN              14 //instr + 4 byte ID + dlc + 8 data

struct can_message_t {
    uint16_t    id;
    uint8_t     len;
    uint8_t     data[8];
};


struct rx_buff_addr_t {
    uint8_t     RXBSIDH;
    uint8_t     RXBSIDL;
    uint8_t     RXBDLC;
    uint8_t     RXBnD;
};

int mcp2515_init(void);
void mcp2515_reset(void);
uint8_t mcp2515_read_status(void);
int mcp2515_send_msg(struct can_message_t *can_msg);
void mcp2515_get_msg(uint8_t num, struct can_message_t *can_msg);
void mcp2515_dump_status(void);

void mcp2515_set_filter(uint8_t id, uint16_t filter, uint16_t mask);

#endif