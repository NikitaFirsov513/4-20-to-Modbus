#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f103xb.h"

void modbus_init(void);

uint8_t modbus_read_frame(uint8_t *frame, uint16_t len, uint8_t *tx, uint16_t *tx_len);

uint8_t modbus_read_holding_registers(uint8_t slave_addr,
                                      uint16_t start_address,
                                      uint16_t quantity,
                                      uint8_t *tx,
                                      uint16_t *tx_len);

uint8_t modbus_write_single_register(uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t value,
                                     uint8_t *tx,
                                     uint16_t *tx_len);
uint8_t modbus_write_multiple_registers(uint8_t slave_addr,
                                        uint16_t start_address,
                                        uint16_t quantity,
                                        uint8_t data_len,
                                        uint8_t *data,
                                        uint8_t *tx,
                                        uint16_t *tx_len);
void modbus_send_response(uint8_t *frame, uint16_t len);

uint16_t modbus_crc16(const uint8_t *frame, uint16_t len);
bool modbus_verify_crc(const uint8_t *frame, uint16_t len);
