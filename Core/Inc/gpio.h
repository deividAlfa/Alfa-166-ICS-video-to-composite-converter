/*
 * gpio.h
 *
 *  Created on: 20 sept. 2025
 *      Author: David
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/*

 GPIO macros to interface HAL

 For convenience, two types of the same function are provided.

 non-HAL: Independent Pin and Port. x_GPIO_Port/x_Pin, or GPIOx/GPIO_PIN_x

 HAL: Assumes pin named in cube MX, thus the input can be greatly simplified. Example:
     Pin "LED" defined in CubeMX. This leads to the automatic creation of "LED_GPIO_Port" and "LED_Pin" definitions.
     Knowing that, we can simplify to just "LED" and generate the rest.

 No-named pins are only compatible with non-HAL functions:
     writePin(port,pin,val):      writePin(GPIOA, GPIO_PIN_1, SET);
     setPinHigh(port,pin)         setPinHigh(GPIOA, GPIO_PIN_1);
     setPinMode(port,pin,mode)    setPinMode(GPIOA, GPIO_PIN_1, MODE_OUTPUT);
     setPinOtype(port,pin,Otype)  setPinOtype(GPIOA, GPIO_PIN_1, OUTPUT_OD);

 Defined pins can be used with either:
     writePin(port,pin,val):      writePin(LED_GPIO_Port, LED_Pin, SET);
     setPinHigh(port,pin)         setPinHigh(LED_GPIO_Port, LED_Pin);
     setPinMode(port,pin,mode)    setPinMode(LED_GPIO_Port, LED_Pin, MODE_OUTPUT);
     setPinOtype(port,pin,Otype)  setPinOtype(LED_GPIO_Port, LED_Pin, OUTPUT_OD);

     HAL_writePin(name,val)       HAL_writePin(LED, SET);
     HAL_setPinHigh(port,pin)     HAL_setPinHigh(LED);
     HAL_setPinMode(name,mode)    HAL_setPinMode(LED, MODE_OUTPUT);
     HAL_setPinOtype(name,Otype)  HAL_setPinOtype(LED, OUTPUT_OD);
*/

#define _CON2(a,b)        (a##b)
#define _PORT(p)          (_CON2(p,_GPIO_Port))
#define _PIN(p)           (_CON2(p,_Pin))


//   Converts GPIO_PIN_x into pin number
//   0b00000001->0         0b00100000->5
#define __get_GPIO_Pos(x)             (15*(1&&(x&(1<<15)))  + \
                                      14*(1&&(x&(1<<14)))   + \
                                      13*(1&&(x&(1<<13)))   + \
                                      12*(1&&(x&(1<<12)))   + \
                                      11*(1&&(x&(1<<11)))   + \
                                      10*(1&&(x&(1<<10)))   + \
                                      9*(1&&(x&(1<<9)))     + \
                                      8*(1&&(x&(1<<8)))     + \
                                      7*(1&&(x&(1<<7)))     + \
                                      6*(1&&(x&(1<<6)))     + \
                                      5*(1&&(x&(1<<5)))     + \
                                      4*(1&&(x&(1<<4)))     + \
                                      3*(1&&(x&(1<<3)))     + \
                                      2*(1&&(x&(1<<2)))     + \
                                      1*(1&&(x&(1<<1))))

//   Expands 16bit to 32bit.    0b1001->0b11000011
//   Source: https://stackoverflow.com/questions/38881877/bit-hack-expanding-bits
#define __expand_1(x)                 ((x | (x << 8)) & 0x00FF00FF)
#define __expand_2(x)                 ((x | (x << 4)) & 0x0F0F0F0F)
#define __expand_3(x)                 ((x | (x << 2)) & 0x33333333)
#define __expand_4(x)                 ((x | (x << 1)) & 0x55555555)
#define __expand_5(x)                 ( x | (x << 1))
#define __expand_16to32(x)            (__expand_5(__expand_4(__expand_3(__expand_2(__expand_1(x))))))

//    GPIOx_MODER
//    mode: MODE_INPUT, MODE_OUTPUT, MODE_AF, MODE_ANALOG
#define setPinMode(port,pin,mode)       (port->MODER = (port->MODER & ~(__expand_16to32(pin))) | mode<<(__get_GPIO_Pos(pin)*2))
#define HAL_setPinMode(name,mode)       (_PORT(name)->MODER = (_PORT(name)->MODER & ~(__expand_16to32(_PIN(name)))) | mode<<(__get_GPIO_Pos(_PIN(name))*2))

//    GPIOx_OTYPER
//    Otype: OUTPUT_OD, OUTPUT_PP
#define setPinOtype(port,pin,Otype)     (port->OTYPER = (port->OTYPER & ~(pin)) | (pin*Otype))
#define HAL_setPinOtype(name,Otype)     (_PORT(name)->OTYPER = (_PORT(name)->OTYPER & ~(_PIN(name))) | (_PIN(name)*Otype))

//    GPIOx_OSPEEDR
//    speed: GPIO_SPEED_FREQ_LOW, GPIO_SPEED_FREQ_MEDIUM, GPIO_SPEED_FREQ_HIGH, GPIO_SPEED_FREQ_VERY_HIGH
#define setPinSpeed(port,pin,speed)     (port->OSPEEDR = (port->OSPEEDR & ~(__expand_16to32(pin))) | speed<<(__get_GPIO_Pos(pin)*2))
#define HAL_setPinSpeed(name,speed)     (_PORT(name)->OSPEEDR = (_PORT(name)->OSPEEDR & ~(__expand_16to32(_PIN(name)))) | speed<<(__get_GPIO_Pos(_PIN(name))*2))

//    GPIOx_PUPDR
//    pull: GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN
#define setPinPull(port,pin,pull)       (port->PUPDR = (port->PUPDR & ~(__expand_16to32(pin))) | pull<<(__get_GPIO_Pos(pin)*2))
#define HAL_setPinPull(name,pull)       (_PORT(name)->PUPDR = (_PORT(name)->PUPDR & ~(__expand_16to32(_PIN(name)))) | pull<<(__get_GPIO_Pos(_PIN(name))*2))

//    GPIOx_IDR
#define readPort(port)                  (port->IDR)
#define HAL_readPort(name)              (_PORT(name)->IDR)
#define readPin(port,pin)               (1&&(port->IDR & pin))
#define HAL_readPin(name)               (1&&(_PORT(name)->IDR & _PIN(name)))

//    GPIOx_ODR
#define writePort(port,val)             (port->ODR = val)
#define HAL_writePort(name,val)         (_PORT(name)->ODR = val)

//    GPIOx_BSRR
#define writePin(port,pin,val)          (port->BSRR = pin << ((val != 0)*16))
#define HAL_writePin(name,val)          (_PORT(name)->BSRR = _PIN(name) << ((val != 0)*16))
#define setPinHigh(port,pin)            ((port->BSRR = pin)
#define HAL_setPinHigh(name)            (_PORT(name)->BSRR = _PIN(name))
#define setPinLow(port,pin)             (port->BSRR = pin<<16)
#define HAL_setPinLow(name)             (_PORT(name)->BSRR = _PIN(name)<<16)
#define togglePin(port,pin)             (port->BSRR = pin << (((port->ODR & pin) != 0)*16))
#define HAL_togglePin(name)             (_PORT(name)->BSRR = _PIN(name) << (((_PORT(name)->ODR & _PIN(name)) != 0)*16))

#endif /* INC_GPIO_H_ */
