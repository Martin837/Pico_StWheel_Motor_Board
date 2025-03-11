
#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include "martinlib.h"
#include "hardware/adc.h"

uint8_t last_debounce_state[29] = {0};

//initialize values



/// (pin, peripheral, config)
void pinConfig(uint8_t pin, uint8_t peripheral, uint8_t config){
    switch(peripheral){
        case 1: //SPI

        case 2: //UART

        case 3: //I2C

        case 4: //PWM

        case 5: //SIO
            if((config >> 7)&1){ //Input
                iobank0_hw->io[pin].ctrl = peripheral;
                padsbank0_hw->io[pin] = config;
                sio_hw->gpio_oe &= ~(1<<pin);
                break;
            }
            else{ //Output
                iobank0_hw->io[pin].ctrl = peripheral;
                padsbank0_hw->io[pin] = config;
                sio_hw->gpio_oe |= 1<<pin;
                break;
            }
        
        case 6: //PIO0

        case 7: //PIO1

        case 8: //Clock

        case 9: //USB Voltage

        default:
            printf("PinConfig error: invalid mode");
            break;
    }
    return;
}
//Modes: 0-Normal, 1-Edge detect, 2-Debounce, Default-Normal
int readPin(uint8_t pin, uint8_t mode){
    if(mode == 1){
        if(((sio_hw->gpio_in >> pin) & 1) != last_debounce_state[pin]){
            last_debounce_state[pin] = (sio_hw->gpio_in >> pin) & 1;
            return last_debounce_state[pin];
        }
        else
            return last_debounce_state[pin];
    }
    else if(mode == 2){
        //debounce
        return 0;
    }
    else
        return (sio_hw->gpio_in >> pin) & 1;
}

void writePin(uint8_t pin, uint8_t mode, uint8_t value){
    if(mode == 0){
        if(!value){
            sio_hw->gpio_out |= 1<<pin;
            return;
        }
        else{
            sio_hw->gpio_out &= ~(1<<pin);
            return;
        }
    }
}

//Modes: 0-One shot, 1-Continuous 
void begin_adc(uint8_t channel, uint8_t mode){
    adc_hw->cs = ADC_CS_EN_BITS;
    
    switch(channel){
        case 0:
            iobank0_hw->io[26].ctrl = GPIO_FUNC_NULL;
            padsbank0_hw->io[26] =  PADS_BANK0_GPIO0_OD_BITS;
            break;
        
        case 1:
            iobank0_hw->io[27].ctrl = GPIO_FUNC_NULL;
            padsbank0_hw->io[27] =  PADS_BANK0_GPIO0_OD_BITS;
            break;
        
        case 2:
            iobank0_hw->io[28].ctrl = GPIO_FUNC_NULL;
            padsbank0_hw->io[28] =  PADS_BANK0_GPIO0_OD_BITS;
            break;
        default:
            printf("begin_adc error: invalid channel selected");
            return;
    }
    adc_hw->cs =(adc_hw->cs & (~(ADC_CS_AINSEL_BITS))) | channel << ADC_CS_AINSEL_LSB;

    if(mode){
        adc_hw->cs |= ADC_CS_START_MANY_BITS;
    }
    else{
        adc_hw->cs |= ADC_CS_START_ONCE_BITS;
    }
    return;
}

int stop_adc(){
    adc_hw -> cs &= 0b0001;
    return 1;
}

int disable_adc(){
    adc_hw -> cs &= 0b0000;
    return 1;
}

void begin_systick(uint32_t reload){
    if(reload > 16777216){
        printf("Systick begin error: invalid reload value selected");
        return;
    }
    SysTick->LOAD  = reload;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk  |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    return;
}

//interrupts page 238