#include "mcp2515.h"


static uint8_t mcp2515_read_reg(uint8_t reg);
static uint8_t mcp2515_write_reg(uint8_t reg, uint8_t data);
static uint8_t mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data);
static uint8_t mcp2515_start_tx(uint8_t num);
//static uint8_t mcp2515_read_status(void);
static uint8_t mcp2515_read_rx_status(void);

volatile uint8_t __attribute__ ((aligned (4))) tx_buffer[SPI_TRANS_LEN];
volatile uint8_t __attribute__ ((aligned (4))) rx_buffer[SPI_TRANS_LEN];


volatile struct can_message_t can_msg_buffer[CAN_MSG_BUFFER_SIZE];
volatile uint32_t can_msg_buffer_index;

const struct rx_buff_addr_t rx0_buff = {
    .RXBSIDH = MCP2515_REG_RXB0SIDH,
    .RXBSIDL = MCP2515_REG_RXB0SIDL,
    .RXBDLC = MCP2515_REG_RXB0DLC,
    .RXBnD = MCP2515_REG_RXB0DATA
};

const struct rx_buff_addr_t rx1_buff = {
    .RXBSIDH = MCP2515_REG_RXB1SIDH,
    .RXBSIDL = MCP2515_REG_RXB1SIDL,
    .RXBDLC = MCP2515_REG_RXB1DLC,
    .RXBnD = MCP2515_REG_RXB1DATA
};


int mcp2515_init(void){

    spi_init();
    mcp2515_reset();
    
    delay_ms(100); //128 OSC CLK minimum

    //clear msg buffer
    can_msg_buffer_index = 0;
    for(uint32_t i=0; i<CAN_MSG_BUFFER_SIZE*sizeof(struct can_message_t);i++){
        *((uint8_t *)can_msg_buffer) = 0;
    }

    mcp2515_dump_status();

    NRF_LOG_INFO("Set CNF values");
    mcp2515_write_reg(MCP2515_REG_CNF1,MCP2515_8MHz_500kBPS_CFG1);
    mcp2515_write_reg(MCP2515_REG_CNF2,MCP2515_8MHz_500kBPS_CFG2);
    mcp2515_write_reg(MCP2515_REG_CNF3,MCP2515_8MHz_500kBPS_CFG3);

    mcp2515_modify_reg(MCP2515_REG_RXB0CTRL,0x4,4); //Enable rollover
    
    mcp2515_write_reg(MCP2515_REG_CANINTE,MCP2515_CANINTE_RX0IE|MCP2515_CANINTE_RX1IE);

    mcp2515_dump_status();
    
    NRF_LOG_INFO("Set normal mode");
    mcp2515_modify_reg(MCP2515_REG_CANCTRL,0xE0,0);

    mcp2515_dump_status();

    return 0;
}


void mcp2515_reset(void){
    *tx_buffer = MCP2515_INSTR_RESET;
    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer,1);
};


void mcp2515_dump_status(void){
    uint8_t data;

    data = mcp2515_read_reg(MCP2515_REG_CANCTRL);
    NRF_LOG_INFO("[CANCTRL] REQOP: %d ABAT: %d OSM: %d CLKEN: %d CLKPRE: %d",((data & 0xE0) >> 5), ((data & 0x10) >> 4), ((data & 0x8) >> 3), ((data & 0x4) >> 2), (data & 0x3) );

    data = mcp2515_read_reg(MCP2515_REG_CANSTAT);
    NRF_LOG_INFO("[CANSTAT] OPMOD: %d ICOD: %d",((data & 0xE0) >> 5),((data & 0xE) >> 1));

    // data = mcp2515_read_status();
    // NRF_LOG_INFO("[READ ST] TX2IF: %d TX2REQ: %d TX1IF: %d TX1REQ: %d TX0IF: %d TX0REQ: %d RX1IF: %d RX0IF: %d",(data & (1<<7)),(data & (1<<6)),(data & (1<<5)),(data & (1<<4)),(data & (1<<3)),(data & (1<<2)),(data & (1<<1)),(data & 1));

    data = mcp2515_read_rx_status();
    NRF_LOG_INFO("[RX STATUS] RECV: %d Msg: %d Filter: %d", ((data & 0xC0) >> 6), ((data & 0x18) >> 3), (data & 0x7) );

};


void mcp2515_send_msg(struct can_message_t *can_msg){
    uint8_t reg;
    uint16_t id = can_msg->id & CAN_STDID_MASK;

    if(can_msg->len > 8){
        NRF_LOG_INFO("CAN PACKET DLC > 8 bytes - %d",can_msg->len);
    };

    //1 Check if TXREQ bit
    reg = mcp2515_read_reg(MCP2515_REG_TXB0CTRL);

    if(reg & MCP2515_TXBnCTRL_TXREQ){
        NRF_LOG_INFO("Pending transmission");
        return;
    }

    //2 Clear: ABTF (TXBnCTRL[6]) MLOA (TXBnCTRL[5]) TXERR (TXBnCTRL[4])
    reg &= ~(MCP2515_TXBnCTRL_ABTF|MCP2515_TXBnCTRL_MLOA|MCP2515_TXBnCTRL_TXERR);
    mcp2515_write_reg(MCP2515_REG_TXB0CTRL,reg);

    //Write ID
    reg = ((id & 7) << 5);
    mcp2515_write_reg(MCP2515_REG_TXB0SIDL,reg);

    reg = (id >> 3) & 0xFF;
    mcp2515_write_reg(MCP2515_REG_TXB0SIDH,reg);

    //Write DLC
    reg = can_msg->len; 
    mcp2515_write_reg(MCP2515_REG_TXB0DLC,reg);

    //Write data bytes
    for(uint32_t i=0; i<can_msg->len; i++){
        mcp2515_write_reg(MCP2515_REG_TXB0DATA+(uint8_t)i,can_msg->data[i]);
    }


    //Start trans on selected TX buff
    mcp2515_start_tx(0);
};


void mcp2515_get_msg(uint8_t num, struct can_message_t *can_msg){
    //struct rx_buff_addr_t *rx_buff;
    //uint8_t data;
    uint8_t out_buf[16];
    uint8_t in_buf[16];
    
    if(num > 1){
        NRF_LOG_INFO("RX Buff index error: %d > 1",num);
    }

    if(num == 1){
        out_buf[0] = MCP2515_INSTR_READ_RX1;
        //rx_buff = (struct rx_buff_addr_t *)&rx1_buff;
    }else{
        out_buf[0] = MCP2515_INSTR_READ_RX0;
        //rx_buff = (struct rx_buff_addr_t *)&rx0_buff;
    }

    spi_transfer((uint8_t *)out_buf, (uint8_t *)in_buf, 13);
    can_msg->id = (in_buf[1] << 3);
    can_msg->id |= ((in_buf[2] >> 5) & 7);
    can_msg->id &= 0x7FF;
    can_msg->len = in_buf[5];

    if(can_msg->len > 8){
        NRF_LOG_ERROR("RX PKT LEN %d",can_msg->len);
    }

    for(uint32_t i=0; i<can_msg->len; i++){
        can_msg->data[i] = in_buf[6+i];
    }

    //data = mcp2515_read_reg(rx_buff->RXBSIDH);
    // can_msg->id = (data << 3);
    // data = mcp2515_read_reg(rx_buff->RXBSIDL);
    
    // can_msg->id |= ((data >> 5) & 7);
    // can_msg->id &= 0x7FF;

    // can_msg->len = mcp2515_read_reg(rx_buff->RXBDLC);

    // if(can_msg->len > 8){
    //     NRF_LOG_INFO("RX PKT LEN %d",can_msg->len);
    // }

    // for(uint32_t i=0; i<can_msg->len; i++){
    //     can_msg->data[i] = mcp2515_read_reg(rx_buff->RXBnD+(uint8_t)i);
    // }

    //mcp2515_modify_reg(MCP2515_REG_CANINTF,(1 << num),0);
}


static uint8_t mcp2515_read_reg(uint8_t reg){
    tx_buffer[0] = MCP2515_INSTR_READ;
    tx_buffer[1] = reg;

    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 3);

    return rx_buffer[2];
};


static uint8_t mcp2515_write_reg(uint8_t reg, uint8_t data){

    tx_buffer[0] = MCP2515_INSTR_WRITE;
    tx_buffer[1] = reg;
    tx_buffer[2] = data;

    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 3);

    return 0;
};


static uint8_t mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data){

    tx_buffer[0] = MCP2515_INSTR_BITMOD;
    tx_buffer[1] = reg;
    tx_buffer[2] = mask;
    tx_buffer[3] = data;

    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 4);

    return 0;
};


static uint8_t mcp2515_start_tx(uint8_t num){
    if(num > 2){
        NRF_LOG_INFO("TX buffer index error. %d > 2",num);
        return 0;
    }

    tx_buffer[0] = MCP2515_INSTR_RTS_TX0|(1 << num);
    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1);
    return 0;
};


uint8_t mcp2515_read_status(void){
    tx_buffer[0] = MCP2515_INSTR_READ_STATUS;
    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 2);
    return rx_buffer[1];
};

static uint8_t mcp2515_read_rx_status(void){
    tx_buffer[0] = MCP2515_INSTR_RX_STATUS;
    spi_transfer((uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 2);
    return rx_buffer[1];
};


int mcp2515_push_msg(struct can_message_t *can_msg){

    if(can_msg_buffer_index >= CAN_MSG_BUFFER_SIZE){
        NRF_LOG_ERROR("CAN ORV");
        return -1;
    }

    can_msg_buffer[can_msg_buffer_index] = *can_msg;
    can_msg_buffer_index++;

    return can_msg_buffer_index;
}

int mcp2515_pull_msg(struct can_message_t *can_msg){

    if(can_msg_buffer_index <= 0){
        return -1;
    }

    *can_msg = can_msg_buffer[can_msg_buffer_index-1];
    can_msg_buffer_index--;

    return can_msg_buffer_index;
}