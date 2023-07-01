/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty. 
 * In no event will the authors be held liable for any damages arising from 
 * the use of this software. Permission is granted to anyone to use this 
 * software for any purpose, including commercial applications, and to alter 
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not 
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be 
 *    appreciated but is not required. 
 * 
 * 2. Altered source versions must be plainly marked as such, and must not be 
 *    misrepresented as being the original software. 
 * 
 * 3. This notice may not be removed or altered from any source distribution.
 *   
 *
 * $Date:        15. April 2014
 * $Revision:    V1.00
 *  
 * Project:      Flash Programming Functions for Texas Instruments TM4C129 Flashes
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "..\FlashOS.H"        // FlashOS Structures


#define HWREG(x)                (*((volatile unsigned long *)(x)))

#define FLASH_USECRL            0x400FE140  // USec Reload
#define FLASH_RMCTL             0x400FE0F0  // ROM Control

#define FLASH_FMA               0x400FD000  // Flash Memory Address
#define FLASH_FMD               0x400FD004  // Flash Memory Data
#define FLASH_FMC               0x400FD008  // Flash Memory Control
#define FLASH_FCRIS             0x400FD00C  // Flash Controller Raw Interrupt
#define FLASH_FCMISC            0x400FD014  // Flash Controller Masked
#define FLASH_FMC2              0x400FD020  // Flash Memory Control 2
#define FLASH_FWBVAL            0x400FD030  // Flash Write Buffer Valid
#define FLASH_FWBN              0x400FD100  // Flash Write Buffer n


// FLASH_FCRIS register definitions
#define FLASH_FCRIS_PROGRAM     0x00000002  // Programming status
#define FLASH_FCRIS_ACCESS      0x00000001  // Invalid access status

// FLASH_FCIM register definitions
#define FLASH_FCIM_PROGRAM      0x00000002  // Programming mask
#define FLASH_FCIM_ACCESS       0x00000001  // Invalid access mask

// FLASH_FCMISC register definitions
#define FLASH_FCMISC_PROGRAM    0x00000002  // Programming status
#define FLASH_FCMISC_ACCESS     0x00000001  // Invalid access status

// FLASH_FMC register definitions
#define FLASH_FMC_WRKEY         0xA4420000  // FLASH write key
#define FLASH_FMC_COMT          0x00000008  // Commit Register Value
#define FLASH_FMC_MERASE        0x00000004  // Mass Erase Flash Memory
#define FLASH_FMC_ERASE         0x00000002  // Erase a Page of Flash Memory
#define FLASH_FMC_WRITE         0x00000001  // Write a Word into Flash Memory

// FLASH_FMC2 register definitions
#define FLASH_FMC2_WRKEY        0xA4420000  // FLASH write key
#define FLASH_FMC2_WRBUF        0x00000001  // Buffered Flash Memory Write

#define FLASH_ERASE_SIZE        0x00000400

#define SYSCTL_NVMSTAT          0x400FE1A0  // Non-Volatile Memory Information

// SYSCTL_NVMSTAT register definitions
#define SYSCTL_NVMSTAT_FWB      0x00000001  // 32 Word Flash Write Buffer


#define STACK_SIZE   64        // Stack Size


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  // Set the Number of Clocks per microsecond for the Flash Controller
  // Approximate division by 1000000 (no Library Code)
  HWREG(FLASH_USECRL) = ((1074*(clk >> 10)) >> 20) - 1;  // clk / 1000000 - 1;

  HWREG(FLASH_RMCTL)  = 0x00000001;                            // Clear the BA bit in RMCTL

  return (0);                                                  // Success
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {
  return (0);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {

  HWREG(FLASH_FCMISC) = FLASH_FCMISC_ACCESS;                   // Clear the Flash Access Interrupt

  HWREG(FLASH_FMC)    = FLASH_FMC_WRKEY | FLASH_FMC_MERASE;    // Mass Erase
  while (HWREG(FLASH_FMC) & FLASH_FMC_MERASE);                 // Wait until Erase is done

  if (HWREG(FLASH_FCRIS) & FLASH_FCRIS_ACCESS) {               // Check Access Violation
    return (1);
  }

  return (0);                                                  // Success
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

  if (adr & (FLASH_ERASE_SIZE - 1)) {                          // Address must be Block aligned  
    return (1);
  }

  HWREG(FLASH_FCMISC) = FLASH_FCMISC_ACCESS;                   // Clear the Flash Access Interrupt

  HWREG(FLASH_FMA) = adr;                                      // Erase the Block
  HWREG(FLASH_FMC) = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
  while (HWREG(FLASH_FMC) & FLASH_FMC_ERASE);                  // Wait until Erase is done

  if (HWREG(FLASH_FCRIS) & FLASH_FCRIS_ACCESS) {               // Wait until Erase is done
    return (1);
  }

  return (0);                                                  // Success
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  if ( (adr & 3) ) {                                           // Address must be Word aligned
    return (1);
  }

  sz = (sz + 3) & ~3;                                          // Adjust size for Words

  HWREG(FLASH_FCMISC) = FLASH_FCMISC_ACCESS;                   // Clear the Flash Access Interrupt

  if(HWREG(SYSCTL_NVMSTAT) & SYSCTL_NVMSTAT_FWB)               // See if this device has a write buffer.
  {
    while(sz) {                                                 // Loop over the words to be programmed
      HWREG(FLASH_FMA)        = adr & ~(0x7f);                 // Set the address of this block of words.

      while(((adr & 0x7c) || (HWREG(FLASH_FWBVAL) == 0)) &&    // Loop over the words in this 32-word block.
            (sz != 0))
      {
        HWREG(FLASH_FWBN + (adr & 0x7c)) = *((unsigned long *)buf); // Write this word into the write buffer.
 
        buf += 4;                                              // Prepeare Next Word
        adr += 4;
        sz  -= 4;
      }
    
      HWREG(FLASH_FMC2) = FLASH_FMC2_WRKEY | FLASH_FMC2_WRBUF; // Program the contents of the write buffer into flash.
      while (HWREG(FLASH_FMC2) & FLASH_FMC2_WRBUF);            // Wait until the write buffer has been programmed.
    }
  }
  else {
    while (sz) {                                               // Loop over the Words to be programmed
  
      HWREG(FLASH_FMA) = adr;                                  // Program Word
      HWREG(FLASH_FMD) = *((unsigned long *)buf);
      HWREG(FLASH_FMC) = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;
      while (HWREG(FLASH_FMC) & FLASH_FMC_WRITE);              // Wait unitl Word has been programmed
  
      adr += 4;                                                // Prepeare Next Word
      buf += 4;
      sz  -= 4;
  
    }
  }

  if (HWREG(FLASH_FCRIS) & FLASH_FCRIS_ACCESS) {               // Check Access Violation
    return (1);
  }

  return (0);                                                  // Success
}
