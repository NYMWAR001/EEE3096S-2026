/* EEE3096S Practical 3: complete the four TODOs in this file.
 * See ../../../docs/prac_03.md and ST RM0091, ADC and DAC chapters.
 * Use CMSIS register names; clock, GPIO and startup helpers are supplied.
 */
#include "voltage_mirror.h"
#include "board_io.h"
#include "main.h"
#include <stddef.h>

#define CODE_MAX        MIRROR_MAX_CODE /* 4095: highest of 4096 codes. */
#define COARSE_MASK     0x0FC0u /* Keep bits 11..6, clear bits 5..0. */
#define DAC_START_CODE  2048u   /* Mid-scale until the first sample arrives. */

/* One conversion = 239.5 + 12.5 = 252 ADC clocks at 14 MHz = 18 us.
 * Each poll is roughly 10 CPU cycles at 48 MHz (~0.2 us), so 1000 polls
 * is ~200 us: about 10x the expected conversion time, yet still well inside
 * the 500 us sampling period, so a stuck ADC can never stall the timer ISR. */
#define EOC_WAIT_POLLS  1000u
#define STOP_WAIT_POLLS 1000u

bool voltage_mirror_init(void)
{
    if (!board_converters_prepare()) return false;

    /* --- ADC1 (configured while ADEN = 0, before board_adc_enable) --- */
    /* Every CFGR1 field we need is 0, so write the whole register:
     * RES = 00: 12-bit.  ALIGN = 0: right-aligned (code in DR[11:0]).
     * EXTEN = 00: software trigger (ADSTART).  CONT = 0: single conversion.
     * DISCEN/AUTOFF/WAIT/DMAEN/OVRMOD = 0: plain polled conversions. */
    ADC1->CFGR1 = 0u;

    /* Plain assignment: channel 5 (PA5) is the ONLY channel in the sequence. */
    ADC1->CHSELR = ADC_CHSELR_CHSEL5;

    /* SMP = 111: 239.5 ADC clock cycles acquisition (longest setting). */
    ADC1->SMPR = ADC_SMPR_SMP;

    /* --- DAC channel 1 (PA4) --- */
    DAC->CR = 0u;                  /* Channel off while it is set up. */
    DAC->DHR12R1 = DAC_START_CODE; /* Mid-scale until the first sample. */
    /* Plain assignment: only EN1 set, every other CR bit 0.
     * BOFF1 = 0 means the output buffer is ON (the bit is "buffer OFF").
     * TEN1 = 0: no trigger, DHR -> DOR one APB clock after each write.
     * TSEL1 = 000, DMAEN1 = 0. */
    DAC->CR = DAC_CR_EN1;

    /* Supplied helper: clears ADRDY (W1C), sets ADEN and waits for ADRDY. */
    return board_adc_enable();
}

bool voltage_mirror_read(uint16_t *code)
{
    if (code == NULL) return false;

    /* ISR flags are write-one-to-clear: write 1s to the flags to clear them.
     * Plain '=' (not '|=') so no other flag, e.g. ADRDY, is cleared by accident. */
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

    /* Start one software-triggered conversion. */
    ADC1->CR |= ADC_CR_ADSTART;

    /* Bounded wait for end of conversion. */
    for (uint32_t remaining = EOC_WAIT_POLLS; remaining != 0u; --remaining)
    {
        if ((ADC1->ISR & ADC_ISR_EOC) != 0u)
        {
            /* Reading DR also clears EOC in hardware. Mask to 12 bits. */
            *code = (uint16_t)(ADC1->DR & CODE_MAX);
            return true;
        }
    }

    /* Timeout: abort the conversion (ADSTP only valid while ADSTART = 1),
     * wait (bounded) for the stop to finish, and report failure.
     * *code is left untouched: we never return a fake result. */
    if ((ADC1->CR & ADC_CR_ADSTART) != 0u)
    {
        ADC1->CR |= ADC_CR_ADSTP;
        for (uint32_t remaining = STOP_WAIT_POLLS; remaining != 0u; --remaining)
        {
            if ((ADC1->CR & ADC_CR_ADSTP) == 0u) break;
        }
    }
    return false;
}

void voltage_mirror_write(uint16_t code)
{
    if (code > CODE_MAX) code = CODE_MAX; /* Saturate, never wrap. */
    DAC->DHR12R1 = code;                  /* 12-bit right-aligned holding reg. */
}

uint16_t voltage_mirror_map(uint16_t code, bool coarse)
{
    if (code > CODE_MAX) code = CODE_MAX;

    /* FULL12:  D = C.
     * COARSE6: D = 64 * floor(C / 64) -- clear the 6 LSBs, stay on the
     * 12-bit DAC scale. 64 commands: 0, 64, ..., 4032 (= 4095 & 0xFC0). */
    return coarse ? (uint16_t)(code & COARSE_MASK) : code;
}
