/******************************************************************************/
/** :Modbus                                                     **/
/**     :V1.0.1                                                         **/
/**     :Modbus **/
/**           1,                                      **/
/**           2,                                  **/
/**           3,Parameters,                                  **/
/**           2,,,Returns    **/
/**                                                                          **/
/* MBAP:                                                       */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |           |     |  |  |                          | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* | | 2 |    |    |/     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |   | 2 |    |    |0=MODBUS                  | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |         | 2 |    |    |                | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |   | 1 |    |    |        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |       | 1 |    |    |                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |         | N |    |    |Returns        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* RTU:                                                        */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |   | 1 |    |    |        | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |       | 1 |    |    |                  | */
/* +-------------+---------+--------+---------------------------------------+ */
/* |         | N |    |    |Returns        | */
/* +-------------+---------+--------+--------+------------------------------+ */
/* |    CRC      | 2 |    |    |CRC     | */
/* +-------------+---------+--------+--------+------------------------------+ */
/**--------------------------------------------------------------------------**/

#ifndef __BSP_MODBUS_H_
#define __BSP_MODBUS_H_

#include <stddef.h>
#include <stdint.h>

#define MODBUS_RESGISTER_ACCESS_MAX_NUM 20  //
#define MODBUS_COLI_STATUS_ACCESS_MAX_NUM \
  (MODBUS_RESGISTER_ACCESS_MAX_NUM << 4)  //

/*Modbus*/
typedef enum {
  MODBUS_RTU,    // RTU transmission mode.
  MODBUS_ASCII,  // ASCII transmission mode.
  MODBUS_TCP     // TCP mode.
} ModbusMode;

/*Modbus,01,02,03,04,05,06,15,16*/
typedef enum {
  MODBUS_ReadCoilStatus = 0x01, /*(Output)*/
  MODBUS_ReadInputStatus = 0x02, /*()*/
  MODBUS_ReadHoldingRegister = 0x03, /*()*/
  MODBUS_ReadInputRegister = 0x04, /*()*/
  MODBUS_WriteSingleCoil = 0x05, /*(Output)*/
  MODBUS_WriteSingleRegister = 0x06, /*()*/
  MODBUS_WriteMultipleCoil = 0x0F, /*(Output)*/
  MODBUS_WriteMultipleRegister =
      0x10, /*()*/
  MODBUS_ReadFileRecord = 0x14,         /**/
  MODBUS_WriteFileRecord = 0x15,        /**/
  MODBUS_MaskWriteRegister = 0x16,      /**/
  MODBUS_ReadWriteMultiRegister = 0x17, /**/
  MODBUS_ReadDeviceID = 0x2B            /**/
} ModbusFunctionCode;

/*()*/
typedef struct __ModbusAccessInfo_t {
  uint8_t ch;       // modbus(modbus)
  ModbusMode mode;  //
  uint8_t unitID;
  ModbusFunctionCode functionCode;
  uint16_t startingAddress;
  uint16_t quantity;
} ModbusAccessInfo_t;

/**/

uint16_t BSP_CalCrc16(uint8_t *data, uint16_t len);

/*()MB,Returns*/
uint16_t BSP_MODBUS_ConvertBoolArrayToMBByteArray(bool *sData, uint16_t length,
                                                  uint8_t *oData);
/**
 * @brief 
 * @param tAccessInfo 
 * @param bStatusList ()
 * @param u16RegisterList ()
 * @param u8CommandBuf 
 * @return ,0
 */
uint16_t BSP_MODBUS_GetReadWriteServerCommand(ModbusAccessInfo_t *tAccessInfo,
                                              bool *bStatusList,
                                              uint16_t *u16RegisterList,
                                              uint8_t *u8CommandBuf);

/**/

#endif
