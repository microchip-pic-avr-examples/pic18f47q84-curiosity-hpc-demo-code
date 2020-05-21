/**
  ADCC Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    adcc.c

  @Summary
    This is the generated driver implementation file for the ADCC driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides implementations for driver APIs for ADCC.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.0
        Device            :  PIC18F47Q84
        Driver Version    :  2.1.4
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.20
        MPLAB             :  MPLAB X 5.40
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <xc.h>
#include "adcc.h"

void ADCC_Initialize(void)
{ 
    ADLTHL = 0x00; 
    ADLTHH = 0x00;
    ADUTHL = 0x00;
    ADUTHH = 0x00; 
    ADSTPTL = 0x00;
    ADSTPTH = 0x00; 
    ADACCU = 0x00; 
    ADRPT = 0x00;    
    ADPCH = 0x00;                                                               // PCH ANA0; 
    ADACQL = 0x00; 
    ADACQH = 0x00;     
    ADCAP = 0x00;                                                               // CAP Additional uC disabled; 
    ADPREL = 0x00;
    ADPREH = 0x00;    
    ADCON1 = 0x00;                                                              // ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss;     
    ADCON2 = 0x00;                                                              // ADCRS 0; ADMD Basic_mode; ADACLR disabled; ADPSIS RES;     
    ADCON3 = 0x00;                                                              // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
    ADSTAT = 0x00;     
    ADREF = 0x00;                                                               // ADNREF VSS; ADPREF VDD;     
    ADACT = 0x00;                                                               // ADACT disabled;     
    ADCLK = 0x00;                                                               // ADCS FOSC/2; 
    ADCON0 = 0x80;                                                              // ADGO stop; ADFM left; ADON enabled; ADCS FOSC/ADCLK; ADCONT disabled;
}

void ADCC_StartConversion(adcc_channel_t channel)
{   
    ADPCH = channel;                                                            // select the A/D channel         
    ADCON0bits.ADON = 1;                                                        // Turn on the ADC module    
    ADCON0bits.ADGO = 1;                                                        // Start the conversion
}

bool ADCC_IsConversionDone(void)
{
    return ((unsigned char)(!ADCON0bits.ADGO));                                 // Start the conversion
}

adc_result_t ADCC_GetConversionResult(void)
{
    return ((adc_result_t)((ADRESH << 8) + ADRESL));                            // Return the result
}

adc_result_t ADCC_GetSingleConversion(adcc_channel_t channel)
{
    ADPCH = channel;                                                            // select the A/D channel
    ADCON0bits.ADON = 1;                                                        // Turn on the ADC module
    ADCON0bits.ADCONT = 0;                                                      // Disable the continuous mode. 
    ADCON0bits.ADGO = 1;                                                        // Start the conversion
    
    while (ADCON0bits.ADGO)                                                     // Wait for the conversion to finish
    {
        CLRWDT();
    }
    
    return ((adc_result_t)((ADRESH << 8) + ADRESL));                            // Conversion finished, return the result
}

void ADCC_StopConversion(void)
{
    ADCON0bits.ADGO = 0;                                                        // Reset the ADGO bit.
}

void ADCC_SetStopOnInterrupt(void)
{
    ADCON3bits.ADSOI = 1;                                                       // Set the ADSOI bit.
}

void ADCC_DischargeSampleCapacitor(void)
{
    ADPCH = 0x3b;                                                               // Set the ADC channel to AVss. 
}

void ADCC_LoadAcquisitionRegister(uint16_t acquisitionValue)
{
    ADACQH = acquisitionValue >> 8;                                             // Load the ADACQH and ADACQL registers.
    ADACQL = acquisitionValue;  
}

void ADCC_SetPrechargeTime(uint16_t prechargeTime)
{
    ADPREH = prechargeTime >> 8;                                                // Load the ADPREH and ADPREL registers.
    ADPREL = prechargeTime;
}

void ADCC_SetRepeatCount(uint8_t repeatCount)
{
    ADRPT = repeatCount;                                                        // Load the ADRPT register.
}

uint8_t ADCC_GetCurrentCountofConversions(void)
{
    return ADCNT;                                                               // Return the contents of ADCNT register
}

void ADCC_ClearAccumulator(void)
{
    ADCON2bits.ADACLR = 1;                                                      // Reset the ADCON2bits.ADACLR bit.
}

uint24_t ADCC_GetAccumulatorValue(void)
{    
    return (((uint24_t)ADACCU << 16)+((uint24_t)ADACCH << 8) + ADACCL);         // Return the contents of ADACCU, ADACCH and ADACCL registers
}

bool ADCC_HasAccumulatorOverflowed(void)
{
    return ADSTATbits.ADAOV;                                                    // Return the status of ADSTATbits.ADAOV
}

uint16_t ADCC_GetFilterValue(void)
{
    return ((uint16_t)((ADFLTRH << 8) + ADFLTRL));                              // Return the contents of ADFLTRH and ADFLTRL registers
}

uint16_t ADCC_GetPreviousResult(void)
{
    return ((uint16_t)((ADPREVH << 8) + ADPREVL));                              // Return the contents of ADPREVH and ADPREVL registers
}

void ADCC_DefineSetPoint(uint16_t setPoint)
{
    ADSTPTH = setPoint >> 8;                                                    // Sets the ADSTPTH and ADSTPTL registers
    ADSTPTL = setPoint;
}

void ADCC_SetUpperThreshold(uint16_t upperThreshold)
{
    ADUTHH = upperThreshold >> 8;                                               // Sets the ADUTHH and ADUTHL registers
    ADUTHL = upperThreshold;
}

void ADCC_SetLowerThreshold(uint16_t lowerThreshold)
{
    ADLTHH = lowerThreshold >> 8;                                               // Sets the ADLTHH and ADLTHL registers
    ADLTHL = lowerThreshold;
}

uint16_t ADCC_GetErrorCalculation(void)
{
	return ((uint16_t)((ADERRH << 8) + ADERRL));                                // Return the contents of ADERRH and ADERRL registers
}

void ADCC_EnableDoubleSampling(void)
{
    ADCON1bits.ADDSEN = 1;                                                      // Sets the ADCON1bits.ADDSEN
}

void ADCC_EnableContinuousConversion(void)
{
    ADCON0bits.ADCONT = 1;                                                      // Sets the ADCON0bits.ADCONT
}

void ADCC_DisableContinuousConversion(void)
{
    ADCON0bits.ADCONT = 0;                                                      // Resets the ADCON0bits.ADCONT
}

bool ADCC_HasErrorCrossedUpperThreshold(void)
{
    return ADSTATbits.ADUTHR;                                                   // Returns the value of ADSTATbits.ADUTHR bit.
}

bool ADCC_HasErrorCrossedLowerThreshold(void)
{
    return ADSTATbits.ADLTHR;                                                   // Returns the value of ADSTATbits.ADLTHR bit.
}

uint8_t ADCC_GetConversionStageStatus(void)
{
    return ADSTATbits.ADSTAT;                                                   // Returns the contents of ADSTATbits.ADSTAT field.
}
/**
 End of File
*/
