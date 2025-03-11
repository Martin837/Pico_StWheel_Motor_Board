/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "math.h"
#include "pico/stdlib.h"
#include "RP2040.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "martinlib.h"
#include "hardware/i2c.h"
#include "mpu_6050.h"
#include "i2cslave.h"
#include "variables.h"

#define fc1 9
#define fc2 10
#define boton1 11
#define boton2 12
#define boton3 13
#define ledback 0
#define MIN_VAL 100

//----variables de led y PID----//
uint16_t val = 0;
float mean_c = 0, valf = 0;
uint16_t encoder=60;
//----variables de led y PID----//

//----variables de raspberry----//
char caracter_rec;
int grados=0;
uint8_t indice=0,recibo[50];
uint8_t encoders[6] = {0}, n=0, arranca = 0, termina = 0;
uint8_t sfc1 = 0, sfc2 = 0, lfc1 = 0, lfc2 = 0; // state fc, last state fc
uint32_t press1 = 0, press2 = 0;
//uint8_t edge1 = 0, edge2 = 0;
uint8_t send = 0, num = 0; //do we need to send something via UART or not; index 
char trama[5] = {0}; //!B14?
//----variables de raspberry----//

uint8_t boton[15] = {0};

uint8_t confi = 0;

void Recibe_car(); 

int main()
{
    stdio_init_all();
    SystemCoreClockUpdate();

    begin_adc(0, 0);

    uint32_t last_read = 0, times = 0;
    float values = 0;
    pinConfig(4, 5, 0b00100001);

    //*-------------uart------------*//
    // Uart 115200 8N1
    uart0_hw->ibrd = 67;
    uart0_hw->fbrd = 53;
    // Largo de la palabra
    // 11 -> 8 bits
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_WLEN_LSB) | (0b11) << UART_UARTLCR_H_WLEN_LSB;
    // Bits de stop
    // 0 -> 1 bit
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_STP2_LSB) | (0b0) << UART_UARTLCR_H_STP2_LSB;
    // Seleccion de paridad
    // 0 -> Paridad impar
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_EPS_LSB) | (0b0) << UART_UARTLCR_H_EPS_LSB;
    // Enable de paridad
    // 0 -> Paridad desabilitada
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_PEN_BITS) | (0b0) << UART_UARTLCR_H_PEN_BITS;
    // Habilito UART, TX y RX
    uart0_hw->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    // Configuro el pin 0 para la funcion de UART (TX)
    padsbank0_hw->io[16] = padsbank0_hw->io[16] & (~PADS_BANK0_GPIO0_OD_BITS); 
    padsbank0_hw->io[16] = padsbank0_hw->io[16] | PADS_BANK0_GPIO0_IE_BITS;    
    iobank0_hw->io[16].ctrl = GPIO_FUNC_UART;
    // Configuro el pin 1 para la funcion de UART (RX)
    padsbank0_hw->io[17] = padsbank0_hw->io[17] & (~PADS_BANK0_GPIO0_OD_BITS);
    padsbank0_hw->io[17] = padsbank0_hw->io[17] | PADS_BANK0_GPIO0_IE_BITS;   
    iobank0_hw->io[17].ctrl = GPIO_FUNC_UART;

    // habilito la recepcion por interrupciones
    uart0_hw->imsc = UART_UARTIMSC_RXIM_BITS;


    // configuro la interrupcion de la uart
    irq_set_exclusive_handler(UART0_IRQ, Recibe_car);
    irq_set_enabled(UART0_IRQ, true);
    
    //*-------------uart------------*//

    //--------------PID variables--------------
    uint32_t current = 0, last = 0, dt = 0;
    float setpoint = 0.5, error = 0, cumulative = 0, deriv = 0, out_d = 0, last_error = 0;
    uint8_t torque = 100;

    const float kp = 100, ki = 10000, kd = 0.001;

    //--------------PID variables--------------

    //3v3 adc = 1.41v res

    //--------------LEDS variables-------------

    //Configuro salida boton
    pinConfig(fc1,5,0b11100111);
    pinConfig(fc2,5,0b11100111);
    
    iobank0_hw->io[4].ctrl = GPIO_FUNC_PWM;
    padsbank0_hw->io[4] = PADS_BANK0_GPIO0_SLEWFAST_BITS;

    pwm_hw->slice[2].top = 24999;

    // Reinicio el regitro CSR
    pwm_hw->slice[2].csr = 0;

    // Activo la correccion de fase
    pwm_hw->slice[2].csr |= PWM_CH0_CSR_PH_CORRECT_BITS;
    pwm_hw->slice[2].csr |= 0b100;

    // Configuro divisor (bits 11:4 divisor entero, bits 3:0 divisor fraccional)
    pwm_hw->slice[2].div = 2 << 4 | 8;
    
    pwm_hw->slice[2].cc = (24999 / 4);

    // Habilito el PWM
    pwm_hw->slice[2].csr |= PWM_CH0_CSR_EN_BITS;

    //---------------pwm------------------//
    
    iobank0_hw->io[0].ctrl = GPIO_FUNC_PWM;
    padsbank0_hw->io[0] = PADS_BANK0_GPIO0_SLEWFAST_BITS;

    iobank0_hw->io[1].ctrl = GPIO_FUNC_PWM;
    padsbank0_hw->io[1] = PADS_BANK0_GPIO0_SLEWFAST_BITS;

    // Me fijo en la tabla y veo que el gpio 25 esta conectado al slice 4B
    // Configuro el modulo del contador (16 bits maximo)
    pwm_hw->slice[0].top = 31346;

    // Reinicio el regitro CSR
    pwm_hw->slice[0].csr = 0;

    // Activo la correccion de fase
    pwm_hw->slice[0].csr |= PWM_CH0_CSR_PH_CORRECT_BITS;
    pwm_hw->slice[0].csr |= 0b1100;

    // Configuro divisor (bits 11:4 divisor entero, bits 3:0 divisor fraccional)
    pwm_hw->slice[0].div = 19 << 4 | 15;
    
    // Los primeros 16 bits (15:0) son la comparacion del canal A, los segundos 16 bits (31:16) son la comparacion del canal B
    // Como quiero configurar el canal B escribo los 16 bits mas significativos
    // Pongo un 25% de duty
    pwm_hw->slice[0].cc = 0;

    // Habilito el PWM
    pwm_hw->slice[0].csr |= PWM_CH0_CSR_EN_BITS;

    //---------------pwm------------------//

    while(true){ 

        if(confi){
            torque = encoders[2];
        }

        setpoint = (float)(3 * torque/100.0); 
        
        float duty1 = 24999.0*(encoders[3] / 100.0);
        uint16_t dutyint = duty1;
        pwm_hw->slice[2].cc = dutyint;
    
        //------------Mean current------------
        if(adc_hw->cs & ADC_CS_READY_BITS){
            val = adc_hw->result;
            values += (val * 3.3)/4095.0;
            times++;
            adc_hw->cs |= ADC_CS_START_ONCE_BITS;
        }

        if(timer_hw->timelr > (last_read+500000)){
            last_read = timer_hw->timelr;
            mean_c = (float)((values / times)/.47);
            times = 0;
            values = 0;
        }
        //------------Mean current------------
        //-----------------PID----------------
        current = timer_hw->timelr;

        dt = current - last;
        error = setpoint - mean_c;
        cumulative += error * dt;
        deriv = (error - last_error) / dt;

        out_d = kp * error + ki * cumulative + kd * deriv;
        
        out_d += 30;

        if(out_d < 0){
            out_d = 0;
        }
        if(out_d>50){
            out_d = 50;
        }


        last_error = error;
        last = current;
        //-----------------PID----------------

        //-----------------Motor----------------

        float duty = 31346 * (out_d/100.0);

        if(mean_c >= 3){ //E-stop
            duty = 31346 * 0.2;
        }

        if (grados > 5)
            pwm_hw->slice[0].cc = (int)duty << 16;
        
        if(grados < -5)
            pwm_hw->slice[0].cc = (int)duty;
        
        if(grados <= 5 || grados >= -5)
            pwm_hw->slice[0].cc = 0;
        
        //-----------------Motor----------------

        sfc1 = readPin(fc1, 0);
        sfc2 = readPin(fc2, 0);

        if((sfc1 != lfc1) && ((press1 + 10e3) < timer_hw->timelr)){
            press1 = timer_hw->timelr;
            lfc1 = sfc1;
            if(sfc1){
                edge1 = 1;
                //context.send_mem[0] = 1;
            }
        }
        
        if((sfc2 != lfc2) && ((press2 + 10e3) < timer_hw->timelr)){
            press2 = timer_hw->timelr;
            lfc2 = sfc2;
            if(sfc2){
                edge1 = 1;
                //context.send_mem[1] = 1;
            }
        }

        /*
        if(!context.mem_address_written && context.requested){
            context.send_mem[0] = 0;
            context.send_mem[1] = 0;
            edge1 = 0;
            edge2 = 0;
            context.requested = 0;
        }
        */

        /*

        if(send){
            if(uart0_hw->fr & UART_UARTFR_TXFE_BITS){ // Verifico que el puerto de tx esta ok
                uart0_hw->dr = trama[num]; // Manda el caracter correspondiente de la trama

                if (trama[num] != '?'){
                    num++;
                }
                else
                { 
                    num = 0;
                    send = 0;
                }
            }
        }
        */
    }
    return 0;
}

//---------------------raspberry---------------------//
void Recibe_car()
{
    if ((uart0_hw->ris & UART_UARTRIS_RXRIS_BITS) != 0) // Chequeo la interrupcion por RX
    {
        caracter_rec = uart0_hw->dr;
        uart0_hw->icr |= UART_UARTICR_RXIC_BITS; // Limpio la interrupcion de RX
    }


    if(caracter_rec =='!'){
        indice=0;
    }

    recibo[indice++] = caracter_rec;

    // !Saaa-B1-B2-B13-B14-E1100-E2090-E3040-E5001-?

    if(caracter_rec=='?'){
        grados = ((recibo[2]-48)*100)+((recibo[3]-48)*10)+(recibo[4]-48);
        for(int i = 0; i < 14; i++){
            boton[i] = 0;
            
            if(i < 6)
                encoders[i] = 0;
        }

        if(recibo[1]=='-')
            grados *= -1;

        for(int i = 4; i < indice; i++){

            if(recibo[i] == '-'){
                if(!arranca){
                    arranca = i;
                }
                else{
                    termina = i;
                    if(recibo[arranca +1] == 'B'){
                        if((termina - arranca - 1) == 3){
                            n = ((recibo[arranca + 2]-48)*10) + (recibo[arranca+3]-48);
                            boton[n] = 1;
                        }
                        else{
                            boton[recibo[arranca+2] - 48] = 1;
                        }
                    }
                    else if(recibo[arranca +1] == 'E'){
                        encoders[recibo[arranca+2]-48] = ((recibo[arranca+3]-48)*100)+((recibo[arranca+4]-48)*10)+(recibo[arranca+5]-48);
                    }
                    else if(recibo[arranca +1] == 'C'){
                        confi = 1;
                    }
                    arranca = 0;
                    i--;
                    
                }
            }
        }
    }
}
//---------------------raspberry---------------------//