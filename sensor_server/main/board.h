#ifndef _BOARD_H_
#define _BOARD_H_

int16_t board_dht_get_temperature(void);

int16_t board_dht_get_humidity(void);

void board_dht_read(void);

void board_init(void);

#endif