#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"

#define BUFF_SIZE	25

extern uint8_t rx_buff[BUFF_SIZE],flag_u1;
extern uint16_t CH[18];
extern uint8_t rx_jy[40],rx_middle[6];
void GET_xy(int16_t *x,int16_t *y);
typedef struct
{
    uint16_t online;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
        uint8_t e;
        uint8_t f;
        uint8_t g;
        uint8_t h;
    } key;
} remoter_t;
extern remoter_t remoter;
void sbus_frame_parse(remoter_t *remoter, uint8_t *buf);
extern uint16_t CH_pan[18];
void GET_angles(float *Yaw,float *Pitch,float *Roll);
#endif /*__UART_BSP_H__ */

