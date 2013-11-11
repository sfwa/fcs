/**
 *   @file  csl_gpioAux.h
 *
 *   @brief   
 *      This is the GPIO Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the GPIO Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef _CSL_GPIOAUX_H_
#define _CSL_GPIOAUX_H_

#include <ti/csl/csl_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_GPIO_FUNCTION
 @{ */


/** ============================================================================
 *   @n@b CSL_GPIO_getPID
 *
 *   @b Description
 *   @n This function reads the peripheral ID register which identifies the 
 *      scheme of PID encoding, function, rtl id, major id, custom id and minor id.
 *
 *   @b Arguments
     @verbatim
          scheme        Scheme of PID encoding
          function      GPIO function
          rtl           RTL ID of GPIO module 
          major         Major ID of GPIO module 
          custom        Custom ID of GPIO module 
          minor         Minor ID of GPIO module 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_PID_SCHEME,GPIO_PID_FUNC,GPIO_PID_RTL,GPIO_PID_MAJOR,GPIO_PID_CUSTOM,GPIO_PID_MINOR
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint16          function;
        Uint8           scheme, rtl, major, custom, minor;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Get the GPIO Peripheral Identification.
        CSL_GPIO_getPID (hGpio, &scheme, &function, &rtl, &major, &custom, &minor);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_getPID 
(
    CSL_GpioHandle  hGpio, 
    Uint8           *scheme, 
    Uint16          *function, 
    Uint8           *rtl,
    Uint8           *major,
    Uint8           *custom,
    Uint8           *minor
)
{
    Uint32 value = hGpio->PID;

    *scheme     = CSL_FEXT (value, GPIO_PID_SCHEME);
    *function   = CSL_FEXT (value, GPIO_PID_FUNC);
    *rtl        = CSL_FEXT (value, GPIO_PID_RTL);
    *major      = CSL_FEXT (value, GPIO_PID_MAJOR);
    *custom     = CSL_FEXT (value, GPIO_PID_CUSTOM);
    *minor      = CSL_FEXT (value, GPIO_PID_MINOR);
}

/** ============================================================================
 *   @n@b CSL_GPIO_getPCR
 *
 *   @b Description
 *   @n This function reads the peripheral Control register which identifies the 
 *      emulation mode. 
 *
 *   @b Arguments
     @verbatim
          soft          Used in conjunction with FREE bit to determine
                        the emulation suspend mode. GPIO has FREE bit set to 1 
                        so SOFT bit does not affect functionality.
          free          For GPIO, the FREE bit is fixed at 1, which
                        means GPIO runs free in emulation suspend.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_PCR_SOFT,GPIO_PCR_FREE
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           soft, free;
 
        ...
        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);

        // Get the GPIO Peripheral Control register configuration.
        CSL_GPIO_getPCR (hGpio, &soft, &free);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_getPCR 
(
    CSL_GpioHandle  hGpio, 
    Uint8           *soft, 
    Uint8           *free 
)
{
    Uint32 value = hGpio->PCR;
    *soft = CSL_FEXT (value, GPIO_PCR_SOFT);
    *free = CSL_FEXT (value, GPIO_PCR_FREE);
}

/** ============================================================================
 *   @n@b CSL_GPIO_bankInterruptEnable
 *
 *   @b Description
 *   @n This function enables the GPIO per bank interrupt. Each bank supports 16 GPIO signals.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          bankNum           GPIO Bank Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_BINTEN_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           bankNum = 0;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Enable GPIO per bank interrupt for bank zero
        CSL_GPIO_bankInterruptEnable (hGpio, bankNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_bankInterruptEnable 
(
    CSL_GpioHandle  hGpio,
    Uint8           bankNum
)
{
    CSL_FINSR (hGpio->BINTEN, bankNum, bankNum, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_bankInterruptDisable
 *
 *   @b Description
 *   @n This function disables the GPIO per bank interrupt. Each bank supports 16 GPIO signals.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          bankNum           GPIO Bank Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_BINTEN_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           bankNum = 0;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Disable GPIO per bank interrupt for bank zero
        CSL_GPIO_bankInterruptDisable (hGpio, bankNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_bankInterruptDisable 
(
    CSL_GpioHandle  hGpio,
    Uint8           bankNum
)
{
    CSL_FINSR (hGpio->BINTEN, bankNum, bankNum, 0);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_isBankInterruptEnabled
 *
 *   @b Description
 *   @n This function returns the status of GPIO per bank interrupt. Each bank supports 16 GPIO signals.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          bankNum           GPIO Bank Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Interrupt is enabled
 *   @n FALSE - Interrupt is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_BINTEN_EN
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check if GPIO per bank interrupt is enabled or disabled
        if (CSL_GPIO_isBankInterruptEnabled (hGpio) == TRUE)
        {
            // GPIO per bank interrupt is ENABLED
        }
        else
        {
            // GPIO per bank interrupt is DISABLED
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_GPIO_isBankInterruptEnabled
(
    CSL_GpioHandle  hGpio,
    Uint8           bankNum
)
{
    if (CSL_FEXTR (hGpio->BINTEN, bankNum, bankNum) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_GPIO_setPinDirOutput
 *
 *   @b Description
 *   @n This function sets the direction of GPIO pin as an output pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_DIR_DIR=0
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Set GPIO pin number 1 as an output pin
        CSL_GPIO_setPinDirOutput (hGpio, pinNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_setPinDirOutput 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    CSL_FINSR (hGpio->BANK_REGISTERS[bankIndex].DIR, bitPos, bitPos, 0);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_setPinDirInput
 *
 *   @b Description
 *   @n This function sets the direction of GPIO pin as an input pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_DIR_DIR=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Set GPIO pin number 1 as an input pin
        CSL_GPIO_setPinDirInput (hGpio, pinNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_setPinDirInput 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
    CSL_FINSR (hGpio->BANK_REGISTERS[bankIndex].DIR, bitPos, bitPos, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_getPinDirection
 *
 *   @b Description
 *   @n This function gets the direction configuration of GPIO pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 0  - Pin is configured as output pin
 *   @n 1  - Pin is configured as input pin
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_DIR_DIR
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check if pin 1 is configured as input or output pin
        if (CSL_GPIO_getPinDirection (hGpio, pinNum))
        {
            // GPIO pin is configured as INPUT
        }
        else
        {
            // GPIO pin is configured as OUTPUT
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_GPIO_getPinDirection 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    return (CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].DIR, bitPos, bitPos));
}

/** ============================================================================
 *   @n@b CSL_GPIO_getOutputData
 *
 *   @b Description
 *   @n This function gets the output drive state of GPIO pin when it is configured as an output pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
          outData           Bit data when GPIO is configured as output pin
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_OUT_DATA_OUT
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1, outData;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Get the output data on pin 1
        CSL_GPIO_getOutputData (hGpio, pinNum, &outData));
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_getOutputData 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum,
    Uint8           *outData
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    *outData = CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].OUT_DATA, bitPos, bitPos);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_setOutputData
 *
 *   @b Description
 *   @n This function sets the output drive state of GPIO pin when it is configured as an output pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_SET_DATA_SET=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 0;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Set output of GPIO pin number 0 to 1
        CSL_GPIO_setOutputData (hGpio, pinNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_setOutputData 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;

    hGpio->BANK_REGISTERS[bankIndex].SET_DATA = 1 << bitPos;
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_clearOutputData
 *
 *   @b Description
 *   @n This function clears the output drive state of GPIO pin when it is configured as an output pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_CLR_DATA_CLR=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 0;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Clear output of GPIO pin number 0
        CSL_GPIO_clearOutputData (hGpio, pinNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_clearOutputData 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    hGpio->BANK_REGISTERS[bankIndex].CLR_DATA = 1 << bitPos;

    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_getInputData
 *
 *   @b Description
 *   @n This function gets the input bit data on GPIO pin when it is configured as an input pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
          inData            Bit data when GPIO is configured as input pin
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_IN_DATA_IN
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1, inData;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Get the output data on pin 1
        CSL_GPIO_getInputData (hGpio, pinNum, &inData));
        ...  
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_getInputData 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum,
    Uint8           *inData
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    *inData = CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].IN_DATA, bitPos, bitPos);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_setRisingEdgeDetect
 *
 *   @b Description
 *   @n This function sets rising edge interrupt detection for GPIO pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_SET_RIS_TRIG_SETRIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Set interrupt detection on GPIO pin 1 to rising edge
        CSL_GPIO_setRisingEdgeDetect (hGpio, pinNum));
        ...    
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_setRisingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    CSL_FINSR (hGpio->BANK_REGISTERS[bankIndex].SET_RIS_TRIG, bitPos, bitPos, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_clearRisingEdgeDetect
 *
 *   @b Description
 *   @n This function clears rising edge interrupt detection for GPIO pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_CLR_RIS_TRIG_CLRRIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Clear rising edge interrupt detection on GPIO pin 1
        CSL_GPIO_clearRisingEdgeDetect (hGpio, pinNum));
        ...    
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_clearRisingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex;
    
    bankIndex = pinNum / 32;
        
    hGpio->BANK_REGISTERS[bankIndex].CLR_RIS_TRIG = 1 << pinNum;
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_isRisingEdgeDetect
 *
 *   @b Description
 *   @n This function checks if the interrupt detection for GPIO pin is set to rising edge or not.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Interrupt detection is set to rising edge
 *   @n FALSE - Interrupt detection is not set to rising edge
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_SET_RIS_TRIG_SETRIS
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check interrupt detection state on GPIO pin 1 
        if (CSL_GPIO_isRisingEdgeDetect (hGpio, pinNum) == TRUE)
        {
            // Interrupt detection is set to RISING EDGE
        }
        else
        {
            // Interrupt detection is not set to RISING EDGE
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_GPIO_isRisingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    if (CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].SET_RIS_TRIG, bitPos, bitPos) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_GPIO_setFallingEdgeDetect
 *
 *   @b Description
 *   @n This function sets falling edge interrupt detection for GPIO pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_SET_FAL_TRIG_SETFAL=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Set interrupt detection on GPIO pin 1 to falling edge
        CSL_GPIO_setFallingEdgeDetect (hGpio, pinNum));
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_setFallingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    CSL_FINSR (hGpio->BANK_REGISTERS[bankIndex].SET_FAL_TRIG, bitPos, bitPos, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_clearFallingEdgeDetect
 *
 *   @b Description
 *   @n This function clears falling edge interrupt detection for GPIO pin.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_CLR_FAL_TRIG_CLRFAL=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Clear falling edge interrupt detection on GPIO pin 1
        CSL_GPIO_clearFallingEdgeDetect (hGpio, pinNum));
        ...     
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_clearFallingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex;
    
    bankIndex = pinNum / 32;
        
    hGpio->BANK_REGISTERS[bankIndex].CLR_FAL_TRIG =  1 << pinNum;
    return;
}

/** ============================================================================
 *   @n@b CSL_GPIO_isFallingEdgeDetect
 *
 *   @b Description
 *   @n This function checks if the interrupt detection for GPIO pin is set to falling edge or not.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Interrupt detection is set to falling edge
 *   @n FALSE - Interrupt detection is not set to falling edge
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_SET_FAL_TRIG_SETFAL
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check interrupt detection state on GPIO pin 1 
        if (CSL_GPIO_isFallingEdgeDetect (hGpio, pinNum) == TRUE)
        {
            // Interrupt detection is set to FALLING EDGE
        }
        else
        {
            // Interrupt detection is not set to FALLING EDGE
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_GPIO_isFallingEdgeDetect 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    if (CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].SET_FAL_TRIG, bitPos, bitPos) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_GPIO_getInterruptStatus
 *
 *   @b Description
 *   @n This function gets the GPIO pin interrupt status.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 0 - Interrupt has not occurred since last cleared
 *   @n 1 - Interrupt has occurred
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n GPIO_INTSTAT_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check interrupt status on pin 1
        if (CSL_GPIO_getInterruptStatus (hGpio, pinNum) == 0)
        {
            // Interrupt has not occured
        }
        else
        {
            // Interrupt has occured
        }
        ...     
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_GPIO_getInterruptStatus 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex, bitPos;
    
    bankIndex = pinNum / 32;
    bitPos = pinNum % 32;
        
    return (CSL_FEXTR (hGpio->BANK_REGISTERS[bankIndex].INTSTAT, bitPos, bitPos));
}

/** ============================================================================
 *   @n@b CSL_GPIO_clearInterruptStatus
 *
 *   @b Description
 *   @n This function clears the GPIO pin interrupt status.
 *
 *   @b Arguments
     @verbatim
          hGpio             Handle of the GPIO device
          pinNum            GPIO Pin Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_GPIO_open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n GPIO_INTSTAT_STAT=1
 *
 *   @b Example
 *   @verbatim
        CSL_GpioHandle  hGpio;
        Uint8           pinNum = 1;

        // Open the CSL GPIO Module 0
        hGpio = CSL_GPIO_open (0);
        ...
        // Check interrupt status on pin 1
        CSL_GPIO_getInterruptStatus (hGpio, pinNum));
        ...        
        // Clear interrupt status on pin 1
        CSL_GPIO_clearInterruptStatus (hGpio, pinNum));
        ...     
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_GPIO_clearInterruptStatus 
(
    CSL_GpioHandle  hGpio,
    Uint8           pinNum
)
{
    Uint8       bankIndex;
    
    bankIndex = pinNum / 32;
        
    hGpio->BANK_REGISTERS[bankIndex].INTSTAT = 1 << pinNum;
    return;
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_GPIOAUX_H_ */


