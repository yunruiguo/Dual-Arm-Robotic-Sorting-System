/**
 * encoding: gb2312
 */

#ifndef _JHERR_H_
#define _JHERR_H_

#define ERR_SUCC 0                // Call succeeded
#define ERR_FUCTION_CALL_ERROR 2  // Invalid API call,API call exception,controller does not support this call
#define ERR_INVALID_HANDLER -1      // Invalid control handle
#define ERR_INVALID_PARAMETER -2    // Invalid parameter
#define ERR_COMMUNICATION_ERR -3    // Communication connection error
#define ERR_KINE_INVERSE_ERR -4     // Inverse kinematics failed
#define ERR_EMERGENCY_PRESSED -5    // Emergency stop is pressed
#define ERR_NOT_POWERED -6          // Robot is not powered on
#define ERR_NOT_ENABLED -7          // Robot is not enabled
#define ERR_DISABLE_SERVOMODE -8    // Robot is not in servo mode
#define ERR_NOT_OFF_ENABLE -9       // Robot enable is not disabled
#define ERR_PROGRAM_IS_RUNNING -10  // Program is running,operation is not allowed
#define ERR_CANNOT_OPEN_FILE -11    // Cannot open file,file does not exist
#define ERR_MOTION_ABNORMAL -12     // Motion exception occurred
#define ERR_FTP_PREFROM -14         // ftpexception
#define ERR_VALUE_OVERSIZE -15      // socket msg or value oversize

#endif
