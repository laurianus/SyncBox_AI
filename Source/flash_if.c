/**
  ******************************************************************************
  * @file    USB_Host/FWupgrade_Standalone/Src/flash_if.c
  * @author  MCD Application Team
  * @brief   This file provides all the flash layer functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright � 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include "flash_if.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "Device.h"
#include "SystemTimer.h"

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
/* Private macros ------------------------------------------------------------ */

/* Compile-time check: ensure Saved_Cfg struct fits within flash sector 8 (128KB) */
_Static_assert(sizeof(Saved_Cfg) <= (ADDR_FLASH_SECTOR_9 - ADDR_FLASH_SECTOR_8),
               "ERROR: Saved_Cfg struct exceeds flash sector 8 size (128KB)!");

/* Private variables --------------------------------------------------------- */
uint32_t FirstSector = 0;
uint32_t NbOfSectors = 0;
uint32_t SectorError = 0;
uint32_t OB_RDP_LEVEL;



FlagStatus readoutstatusX;

extern Saved_Time_prog savedDatas;
extern Saved_Cfg savedDatasCfg;



uint8_t ERR_Flash_Found_X = 0;
/*
uint64_t enter_time_tstFL = 0;
uint64_t load_time_tstET = 0;
uint64_t load_time_tstST = 0;
uint64_t load_time_tstLT = 0;
*/
uint32_t programcounter = 0;
uint32_t Flash_Adr = APPLICATION_ADDRESS;

/* Private function prototypes ----------------------------------------------- */
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address);
static FLASH_OBProgramInitTypeDef FLASH_OBProgramInitStruct;
static FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

/* Private functions --------------------------------------------------------- */

/**
  * @brief  Unlocks the Flash to enable the flash control register access.
  * @param  None
  * @retval None
  */
void FLASH_If_FlashUnlock(void)
{
  HAL_FLASH_Unlock();
}

/**
  * @brief  Gets Flash readout protection status.
  * @param  None
  * @retval ReadOut protection status
  */
FlagStatus FLASH_If_ReadOutProtectionStatus(void)
{
  FlagStatus readoutstatus = RESET;

  FLASH_OBProgramInitStruct.RDPLevel = OB_RDP_LEVEL;

  HAL_FLASHEx_OBGetConfig(&FLASH_OBProgramInitStruct);

  if (OB_RDP_LEVEL == SET)
  {
    readoutstatus = SET;
  }
  else
  {
    readoutstatus = RESET;
  }

  return readoutstatus;
}

/**
  * @brief  Erases the required FLASH Sectors.
  * @param  Address: Start address for erasing data
  * @retval 0: Erase sectors done with success
  *         1: Erase error
  */
uint32_t FLASH_If_EraseSectors(uint32_t Address)
{
  /* Erase the user Flash area (area defined by APPLICATION_ADDRESS and
   * USER_FLASH_LAST_PAGE_ADDRESS) *** */

  if (Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    /* Get the 1st sector to erase */
    FirstSector = FLASH_If_GetSectorNumber(Address);
    /* Get the number of sector to erase from 1st sector */
    NbOfSectors =
      FLASH_If_GetSectorNumber(USER_FLASH_LAST_PAGE_ADDRESS) - FirstSector + 1;

    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    FLASH_EraseInitStruct.Sector = FirstSector;
    FLASH_EraseInitStruct.NbSectors = NbOfSectors;
    FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &SectorError) != HAL_OK)
      return (1);
  }
  else
  {
    return (1);
  }

  return (0);
}

/**
  * @brief  Writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  Address: Start address for writing data buffer
  * @param  Data: Pointer on data buffer
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  */
uint32_t FLASH_If_Write(uint32_t Address, uint32_t Data)
{
  /* Program the user Flash area word by word (area defined by
   * FLASH_USER_START_ADDR and APPLICATION_ADDRESS) ********** */

  if (Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data) != HAL_OK)
      return (1);
  }
  else
  {
    return (1);
  }

  return (0);
}




/**
  * @brief  Returns the Flash sector Number of the address
  * @param  None
  * @retval The Flash sector Number of the address
  */
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address)
{
  uint32_t sector = 0;

  if (Address < ADDR_FLASH_SECTOR_1 && Address >= ADDR_FLASH_SECTOR_0)
  {
    sector = FLASH_SECTOR_0;
  }
  else if (Address < ADDR_FLASH_SECTOR_2 && Address >= ADDR_FLASH_SECTOR_1)
  {
    sector = FLASH_SECTOR_1;
  }
  else if (Address < ADDR_FLASH_SECTOR_3 && Address >= ADDR_FLASH_SECTOR_2)
  {
    sector = FLASH_SECTOR_2;
  }
  else if (Address < ADDR_FLASH_SECTOR_4 && Address >= ADDR_FLASH_SECTOR_3)
  {
    sector = FLASH_SECTOR_3;
  }
  else if (Address < ADDR_FLASH_SECTOR_5 && Address >= ADDR_FLASH_SECTOR_4)
  {
    sector = FLASH_SECTOR_4;
  }
  else if (Address < ADDR_FLASH_SECTOR_6 && Address >= ADDR_FLASH_SECTOR_5)
  {
    sector = FLASH_SECTOR_5;
  }
  else if (Address < ADDR_FLASH_SECTOR_7 && Address >= ADDR_FLASH_SECTOR_6)
  {
    sector = FLASH_SECTOR_6;
  }
  else if (Address < ADDR_FLASH_SECTOR_8 && Address >= ADDR_FLASH_SECTOR_7)
  {
    sector = FLASH_SECTOR_7;
  }
  else if (Address < ADDR_FLASH_SECTOR_9 && Address >= ADDR_FLASH_SECTOR_8)
  {
    sector = FLASH_SECTOR_8;
  }
  else if (Address < ADDR_FLASH_SECTOR_10 && Address >= ADDR_FLASH_SECTOR_9)
  {
    sector = FLASH_SECTOR_9;
  }
  else if (Address < ADDR_FLASH_SECTOR_11 && Address >= ADDR_FLASH_SECTOR_10)
  {
    sector = FLASH_SECTOR_10;
  }
#if defined(CONFIG_HW_F407)
  else                          /* (Address < FLASH_END_ADDR) && (Address >= *
                                 * ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }
#elif defined(CONFIG_HW_F429)
  else if (Address < ADDR_FLASH_SECTOR_12 && Address >= ADDR_FLASH_SECTOR_11)
  {
    sector = FLASH_SECTOR_11;
  }
  else if (Address < ADDR_FLASH_SECTOR_13 && Address >= ADDR_FLASH_SECTOR_12)
  {
    sector = FLASH_SECTOR_12;
  }
  else if (Address < ADDR_FLASH_SECTOR_14 && Address >= ADDR_FLASH_SECTOR_13)
  {
    sector = FLASH_SECTOR_13;
  }
  else if (Address < ADDR_FLASH_SECTOR_15 && Address >= ADDR_FLASH_SECTOR_14)
  {
    sector = FLASH_SECTOR_14;
  }
  else if (Address < ADDR_FLASH_SECTOR_16 && Address >= ADDR_FLASH_SECTOR_15)
  {
    sector = FLASH_SECTOR_15;
  }
  else if (Address < ADDR_FLASH_SECTOR_17 && Address >= ADDR_FLASH_SECTOR_16)
  {
    sector = FLASH_SECTOR_16;
  }
  else if (Address < ADDR_FLASH_SECTOR_18 && Address >= ADDR_FLASH_SECTOR_17)
  {
    sector = FLASH_SECTOR_17;
  }
  else if (Address < ADDR_FLASH_SECTOR_19 && Address >= ADDR_FLASH_SECTOR_18)
  {
    sector = FLASH_SECTOR_18;
  }
  else if (Address < ADDR_FLASH_SECTOR_20 && Address >= ADDR_FLASH_SECTOR_19)
  {
    sector = FLASH_SECTOR_19;
  }
  else if (Address < ADDR_FLASH_SECTOR_21 && Address >= ADDR_FLASH_SECTOR_20)
  {
    sector = FLASH_SECTOR_20;
  }
  else if (Address < ADDR_FLASH_SECTOR_22 && Address >= ADDR_FLASH_SECTOR_21)
  {
    sector = FLASH_SECTOR_21;
  }
  else if (Address < ADDR_FLASH_SECTOR_23 && Address >= ADDR_FLASH_SECTOR_22)
  {
    sector = FLASH_SECTOR_22;
  }
  else                          /* (Address < FLASH_END_ADDR) && (Address >= *
                                 * ADDR_FLASH_SECTOR_23) */
  {
    sector = FLASH_SECTOR_23;
  }
#endif  // defined(CONFIG_HW_F429)
  return sector;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





void erase_flash_script(void)
{
     HAL_FLASH_Unlock();
     FLASH_Erase_Sector(FLASH_If_GetSectorNumber(APPLICATION_ADDRESS), VOLTAGE_RANGE_3);
     HAL_FLASH_Lock();
}



void save_script_flash(void)
{
        
       savedDatasCfg.AB_address = adress_interface();
        
    //   enter_time_tstFL =  GetCurrentSystemTime(); 
       erase_flash_script();
    //   load_time_tstLT = GetCurrentSystemTime() - enter_time_tstFL;
        
       ERR_Flash_Found_X = save_flash_script();
       ERR_Flash_Found_X = check_flash_script();
       
       
       ERR_Flash_Found_X = load_flash_script();
       
}


uint8_t save_flash_script(void)
{
    uint8_t ERR_Flash_Found = 0;

    uint32_t *data_p = (uint32_t* )&savedDatasCfg;  
    savedDatasCfg.preamble1 = PREAMBLE_1;
    savedDatasCfg.preamble2 = PREAMBLE_2;
        
    Flash_Adr = APPLICATION_ADDRESS;
    
    
    savedDatasCfg.ignore1 = 0x00;
    savedDatasCfg.ignore2 = 0x00;
    
        
    savedDatasCfg.validSavedDataCfg = 1;
        
    ERR_Flash_Found = 0;
    programcounter = 0;    
    
    

    HAL_FLASH_Unlock();
    /* NOTE: Removed single-byte write of 0x11 at Flash_Adr.
     * Flash can only change bits 1->0 without erase. Writing 0x11 then
     * overwriting the same address with struct data caused corruption
     * when the first byte of savedDatasCfg != 0x11. The validSavedDataCfg
     * field + preamble markers are sufficient for validity checking. */

   //4 bytes write
   uint32_t WordCount = (sizeof(savedDatasCfg) + 3) / 4;
        
    for (programcounter = 0; programcounter < WordCount; programcounter++)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Flash_Adr, *data_p) != HAL_OK)
      {
          ERR_Flash_Found = 1;
      }
      data_p += 1;
      Flash_Adr += 4;
      
    }
    
    HAL_FLASH_Lock();
    return ERR_Flash_Found;
}

uint8_t check_flash_script(void)
{
     uint8_t ERR_Flash_Found = 0;

     ERR_Flash_Found = 0;
     programcounter = 0;
     Flash_Adr = APPLICATION_ADDRESS;

    /* NOTE: Flash unlock/lock removed — reading flash does not require unlock.
     * Leaving flash unlocked during reads creates a window for accidental writes. */

//compare 4 bytes
    uint32_t WordCount = (sizeof(savedDatasCfg) + 3) / 4;
    uint32_t *data_r = (uint32_t* )&savedDatasCfg;

    for (programcounter = 0; programcounter < WordCount; programcounter++, data_r++, Flash_Adr += 4)
    {
      if(*data_r != *(uint32_t*)Flash_Adr)
      {
                ERR_Flash_Found = 1;
      }
    }

    return ERR_Flash_Found;
}

uint8_t load_flash_script(void)
{

     uint8_t ERR_Flash_Found = 0;
     memset((uint8_t*)&savedDatasCfg, 0x00, sizeof(savedDatasCfg));



     programcounter = 0;
     Flash_Adr = APPLICATION_ADDRESS;
     ERR_Flash_Found = 0;


    //load 4 bytes - 8ms
    uint32_t *data_r = (uint32_t* )&savedDatasCfg;

    /* NOTE: Flash unlock/lock removed — reading flash does not require unlock. */

    uint32_t WordCount = (sizeof(savedDatasCfg) + 3) / 4;

    for (programcounter = 0; programcounter < WordCount; programcounter++, data_r++, Flash_Adr += 4)
    {
      *data_r = *(uint32_t*)Flash_Adr;
    }


   if( savedDatasCfg.preamble1 == PREAMBLE_1 && savedDatasCfg.preamble2 == PREAMBLE_2)
   {
        ERR_Flash_Found = 0;
   }
   else
   {
        ERR_Flash_Found = 1;
        memset((uint8_t*)&savedDatasCfg, 0x00, sizeof(savedDatasCfg));
   }
  // set_valid_script_pos();
  
   return ERR_Flash_Found;
}


