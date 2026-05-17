	/**
	* @last update Nov 30 2021 
	* @sdkversion 2.1.2.3_dev
	* @Maintenance
	*/
#ifndef _JHTYPES_H_
#define _JHTYPES_H_

#define TRUE 1
#define FALSE 0
#include <stdio.h>
#include <stdint.h>

typedef int BOOL;	 //
typedef int JKHD;	 //
typedef int errno_t; //Returns

/**
* @brief Cartesian position
*/
typedef struct
{
	double x; ///< x,mm
	double y; ///< y,mm
	double z; ///< z,mm
} CartesianTran;

/**
* @brief 
*/
typedef struct
{
	double rx; ///< X,:rad
	double ry; ///< Y,:rad
	double rz; ///< Z,:rad
} Rpy;

/**
* @brief 
*/
typedef struct
{
	double s;
	double x;
	double y;
	double z;
} Quaternion;

/**
 *@brief 
 */
typedef struct
{
	CartesianTran tran; ///< Cartesian position
	Rpy rpy;			///< 
} CartesianPose;

/**
* @brief 
*/
typedef struct
{
	CartesianTran x; ///< x
	CartesianTran y; ///< y
	CartesianTran z; ///< z
} RotMatrix;

/**
* @brief 
*/
typedef enum
{
	PROGRAM_IDLE,	 ///< 
	PROGRAM_RUNNING, ///< 
	PROGRAM_PAUSED	 ///< 
} ProgramState;

/**
* @brief 
*/
typedef enum
{
	COORD_BASE,	 ///< 
	COORD_JOINT, ///< 
	COORD_TOOL	 ///< 
} CoordType;

/**
* @brief jog 
*/
typedef enum
{
	ABS = 0, ///< 
	INCR,	 ///< 
	CONTINUE ///< 
} MoveMode;

/**
* @brief 
*/
typedef struct
{
	int scbMajorVersion;		///<scb
	int scbMinorVersion;		///<scb
	int cabTemperature;			///<
	double robotAveragePower;	///<
	double robotAverageCurrent; ///<
	double instCurrent[6];		///<6
	double instVoltage[6];		///<6
	double instTemperature[6];	///<6
} SystemMonitorData;

/**
* @brief 
*/
typedef struct
{
	double mass;			///<,:kg
	CartesianTran centroid; ///<, :mm
} PayLoad;

/**
* @brief joint position
*/
typedef struct
{
	double jVal[6]; ///< 6joint position,:rad
} JointValue;

/**
* @brief IO
*/
typedef enum
{
	IO_CABINET, ///< IO
	IO_TOOL,	///< IO
	IO_EXTEND,	///< IO
	IO_REALY,   ///< IO,CAB V3DO
	IO_MODBUS_SLAVE, ///< ModbusIO,0
	IO_PROFINET_SLAVE, ///< ProfinetIO,0
	IO_EIP_SLAVE      ///< ETHRENET/IPIO,0
} IOType;

/**
* @brief 
*/
typedef struct
{
	BOOL estoped;	   ///< 
	BOOL poweredOn;	   ///< 
	BOOL servoEnabled; ///< 
} RobotState;

/**
* @brief 
*/
typedef void (*CallBackFuncType)(int);

/**
* @brief 
*/

/**
* @brief 
*/
typedef struct
{
	double instCurrent;		///< 
	double instVoltage;		///< 
	double instTemperature; ///< 
	double instVel;			///<  1.7.0.20
	double instTorq;		///< 
} JointMonitorData;

/**
* @brief EXtio
*/
typedef struct
{
	int din[256];				  ///< din[0]
	int dout[256];				  ///< Outputdout[0]
	float ain[256];				  ///< ain[0]
	float aout[256];			      ///< Outputaout[0]
} Io_group;

/**
* @brief 
*/
typedef struct
{
	double scbMajorVersion;				  ///< scb
	double scbMinorVersion;				  ///< scb
	double cabTemperature;				  ///< 
	double robotAveragePower;			  ///< 
	double robotAverageCurrent;			  ///< 
	JointMonitorData jointMonitorData[6]; ///< 6
} RobotMonitorData;

/**
* @brief 
*/
typedef struct
{
	char ip[20];		 ///< ip
	int port;			 ///< 
	PayLoad payLoad;	 ///< 
	int status;			 ///< 
	int errcode;		 ///< exception
	double actTorque[6]; ///< ()()
	double torque[6];	 ///< 
	double realTorque[6];///< ()
} TorqSensorMonitorData;

/**
* @brief ,get_robot_status
*/
typedef struct
{
	int errcode;									///< error code when the robot reports a runtime error,0means normal operation,other values indicate abnormal operation
	int inpos;										///< ,0,1
	int powered_on;									///< robot power status flag,0means not powered on,1means powered on
	int enabled;									///< robot enable status flag,0means not enabled,1means enabled
	double rapidrate;								///< 
	int protective_stop;							///< whether the robot detected a collision,0means no collision detected,1means collision detected
	int emergency_stop;								///< whether the robot is in emergency stop,0means no emergency stop,1means emergency stop
	int dout[256];									///< Output,dout[0]
	int din[256];									///< ,din[0]	
	double ain[256];								///< ,ain[0]
	double aout[256];								///< Output,aout[0]
	int tio_dout[16];								///< Output,tio_dout[0]
	int tio_din[16];								///< ,tio_din[0]
	double tio_ain[16];								///< ,tio_ain[0]
	int tio_key[3];                                 ///<  [0]free;[1]point;[2]pause_resume;
	Io_group extio;								    ///< IO
	Io_group modbus_slave;							///< Modbus
	Io_group profinet_slave;						///< Profinet
	Io_group eip_slave;								///< Ethernet/IP
	unsigned int current_tool_id;					///< id
	double cartesiantran_position[6];				///< Cartesian position
	double joint_position[6];						///< 
	unsigned int on_soft_limit;						///< ,0,1
	unsigned int current_user_id;					///< id
	int drag_status;								///< ,0,1
	RobotMonitorData robot_monitor_data;			///< 
	TorqSensorMonitorData torq_sensor_monitor_data; ///< 
	int is_socket_connect;							///< sdk,0exception,1
} RobotStatus;

/**
* @brief 
*/
typedef struct
{
	long code;		   ///< 
	char message[120]; ///< 
} ErrorCode;

/**
* @brief Parameters
*/
typedef struct
{
	double xyz_interval; ///< 
	double rpy_interval; ///< 
	double vel;			 ///< 
	double acc;			 ///< 
} TrajTrackPara;

#define MaxLength  256
/**
* @brief 
*/
typedef struct
{
	int len;			 ///< 
	char name[MaxLength][MaxLength]; ///< 
} MultStrStorType;

/**
* @brief Parameters
*/
typedef struct
{
	int executingLineId; ///< id
} OptionalCond;

/**
* @brief exception
*/
typedef enum
{
	MOT_KEEP,  ///< exception
	MOT_PAUSE, ///< exception
	MOT_ABORT  ///< exception
} ProcessType;

/**
* @brief Parameters
*/
typedef struct
{
	int opt;			 ///< , 1 2 3 4 5 6 fx fy fz mx my mz,0
	double ft_user;		 ///< 
	double ft_rebound;	 ///< :
	double ft_constant;	 ///< 
	int ft_normal_track; ///< ,0,1
} AdmitCtrlType;

/**
* @brief Parameters
*/
typedef struct
{
	AdmitCtrlType admit_ctrl[6];
} RobotAdmitCtrl;

/**
* @brief Set
* ,  1>rate1>rate2>rate3>rate4>0
* 1,Setrate1,rate2.rate3,rate40
* 2,Setrate1,rate2,rate3 .rate40
* 3,Set rate1,rate2,rate3,rate4 4
*/
typedef struct
{
	int vc_level; //
	double rate1; //1
	double rate2; //2
	double rate3; //3
	double rate4; //4
} VelCom;

/**
* @brief 
*/
typedef struct
{
	double fx; // x
	double fy; // y
	double fz; // z
	double tx; // x
	double ty; // y
	double tz; // z
} FTxyz;

/**
* @brief ftp
*/
struct FtpFile
{
	const char *filename;
	FILE *stream;
};

/**
 *  @brief DHParameters
 */
typedef struct
{
	double alpha[6];
	double a[6];
	double d[6];
	double joint_homeoff[6];
} DHParam;

/**
 *  @brief rs485Parameters
 */
typedef struct
{
	char sig_name[20];//
	int chn_id;		//RS485ID
	int sig_type;	//
	int sig_addr;	//
	int value;		//  Set
	int frequency;	//10
}SignInfo;

/**
 *  @brief rs485RTUParameters
 */
typedef struct
{
	int chn_id;		//RS485ID  chn_idParameters
	int slaveId;	//SetModbus RTU,ModbusID,
	int baudrate;	//4800,9600,14400,19200,38400,57600,115200,230400
	int databit;	//7,8
	int stopbit;	//1,2
	int parity;		//78->  79-> 69->
}ModRtuComm;

#endif
