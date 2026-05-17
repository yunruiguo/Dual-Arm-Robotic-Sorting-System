/**
* @last update Nov 30 2021 
* @Maintenance
*/



#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include <stdio.h>
#include <string>
#include <stdint.h>
#include "jkerr.h"
#include "jktypes.h"

#if defined(_WIN32) || defined(WIN32)
/**
 * Visual Studio__cpluscplus
 */
#if __cpluscplus

#ifdef DLLEXPORT_API
#undef DLLEXPORT_API
#endif // DLLEXPORT_API

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else // DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllimport)
#endif // DLLEXPORT_EXPORTS

#else // __cpluscplus

#define DLLEXPORT_API

#endif // __cpluscplus

#elif defined(__linux__)

#define DLLEXPORT_API __attribute__((visibility("default")))

#else

#define DLLEXPORT_API

#endif // defined(_WIN32) || defined(WIN32)

class DLLEXPORT_API JAKAZuRobot
{
public:
	/**
	* @brief 
	*/
	JAKAZuRobot();

	/**
	* @brief 
	* @param ip  ip
	* @return ERR_SUCC  
	*/
	errno_t login_in(const char *ip);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t login_out();

	/**
	* @param handle  
	* @return ERR_SUCC  
	*/
	errno_t power_on();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t power_off();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t shut_down();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t enable_robot();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t disable_robot();

	/**
	* @brief 
	* @param aj_num 1_based,0-5,x,y,z,rx,ry,rz
	* @param move_mode ,
	* @param coord_type ,,(/)
	* @param vel_cmd ,rad/s,mm/s
	* @param pos_cmd ,rad,mm
	* @return ERR_SUCC  
	*/
	errno_t jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t jog_stop(int num);

	/**
	* @brief 
	* @param joint_pos 
	* @param move_mode :()
	* @param is_block Set,TRUE FALSE
	* @param speed ,:rad/s
	* @return ERR_SUCC  
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 
	* @param joint_pos 
	* @param move_mode :()
	* @param is_block Set,TRUE FALSE
	* @param speed ,:rad/s
	* @param acc ,:rad/s^2
	* @param tol ,:mm
	* @param option_cond Parameters,,,
	* @return ERR_SUCC  
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc, double tol, const OptionalCond *option_cond);

	/**
	* @brief 
	* @param end_pos 
	* @param move_mode :()
	* @param is_block Set,TRUE  FALSE 
	* @param speed ,:mm/s
	* @return ERR_SUCC  
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 
	* @param end_pos 
	* @param move_mode :()
	* @param is_block Set,TRUE  FALSE 
	* @param speed ,:mm/s
	* @param acc ,mm/s^2
	* @param tol ,mm
	* @param option_cond Parameters,,,
	* @return ERR_SUCC  
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief 
	* @param end_pos 
	* @param mid_pos 
	* @param move_mode :()
	* @param is_block Set,TRUE  FALSE 
	* @param speed ,:rad/s
	* @param acc ,:rad/s^2
	* @param tol , mm
	* @param option_cond Parameters,,,
	* @return ERR_SUCC  
	*/
	errno_t circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond, int circle_cnt = 0);

	/**
	* @brief SERVO MOVE
	* @param enable  TRUESERVO MOVE,FALSE
	* @return ERR_SUCC  
	*/
	errno_t servo_move_enable(BOOL enable);

	/**
	* @brief 
	* @param joint_pos 
	* @param move_mode :
	* @return ERR_SUCC 
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode);

	/**
	* @brief 
	* @param joint_pos 
	* @param move_mode :
	* @param step_num  ,servo_jstep_num*8ms,step_num>=1
	* @return ERR_SUCC 
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief Cartesian position
	* @param cartesian_pose 
	* @param move_mode :
	* @return ERR_SUCC  
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode);

	/**
	* @brief Cartesian position
	* @param cartesian_pose 
	* @param move_mode :
	* @param step_num  ,servo_pstep_num*8ms,step_num>=1
	* @return ERR_SUCC  
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief SetOutput(DO)
	* @param type DO
	* @param index DO
	* @param value DOSet
	* @return ERR_SUCC  
	*/
	errno_t set_digital_output(IOType type, int index, BOOL value);

	/**
	* @brief SetOutput(AO)
	* @param type AO
	* @param index AO
	* @param value AOSet
	* @return ERR_SUCC  
	*/
	errno_t set_analog_output(IOType type, int index, float value);

	/**
	* @brief (DI)
	* @param type DI
	* @param index DI
	* @param result DI
	* @return ERR_SUCC  
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result);

	/**
	* @brief Output(DO)
	* @param type DO
	* @param index DO
	* @param result DO
	* @return ERR_SUCC  
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result);

	/**
	* @brief (AI)
	* @param type AI
	* @param index AI
	* @param result AI
	* @return ERR_SUCC  
	*/
	errno_t get_analog_input(IOType type, int index, float *result);

	/**
	* @brief Output(AO)
	* @param type AO
	* @param index AO
	* @param result AO
	* @return ERR_SUCC  
	*/
	errno_t get_analog_output(IOType type, int index, float *result);

	/**
	* @brief IO
	* @param is_running IO
	* @return ERR_SUCC  
	*/
	errno_t is_extio_running(BOOL *is_running);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t program_run();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t program_pause();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t program_resume();

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t program_abort();

	/**
	* @brief 
	* @param file 
	* @return ERR_SUCC  
	*/
	errno_t program_load(const char *file);

	/**
	* @brief 
	* @param file 
	* @return ERR_SUCC  
	*/
	errno_t get_loaded_program(char *file);

	/**
	* @brief 
	* @param curr_line 
	* @return ERR_SUCC  
	*/
	errno_t get_current_line(int *curr_line);

	/**
	* @brief 
	* @param status 
	* @return ERR_SUCC  
	*/
	errno_t get_program_state(ProgramState *status);

	/**
	* @brief Set
	* @param rapid_rate ,Set[0,1]
	* @return ERR_SUCC  
	*/
	errno_t set_rapidrate(double rapid_rate);

	/**
	* @brief 
	* @param rapid_rate 
	* @return ERR_SUCC  
	*/
	errno_t get_rapidrate(double *rapid_rate);

	/**
	* @brief Set
	* @param id 
	* @param tcp 
	* @param name 
	* @return ERR_SUCC  
	*/
	errno_t set_tool_data(int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief SetID
	* @param id ID
	* @return ERR_SUCC  
	*/
	errno_t set_tool_id(const int id);

	/**
	* @brief ID
	* @param id ID
	* @return ERR_SUCC  
	*/
	errno_t get_tool_id(int *id);

	/**
	* @brief 
	* @param id ID
	* @param tcp 
	* @return ERR_SUCC  
	*/
	errno_t get_tool_data(int id, CartesianPose *tcp);

	/**
	* @brief Set
	* @param id 
	* @param user_frame 
	* @param name 
	* @return ERR_SUCC  
	*/
	errno_t set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief SetID
	* @param id ID
	* @return ERR_SUCC  
	*/
	errno_t set_user_frame_id(const int id);

	/**
	* @brief ID
	* @param id 
	* @return ERR_SUCC  
	*/
	errno_t get_user_frame_id(int *id);

	/**
	* @brief 
	* @param id ID
	* @param tcp 
	* @return ERR_SUCC  
	*/
	errno_t get_user_frame_data(int id, CartesianPose *tcp);

	/**
	* @brief 
	* @param enable  TRUE,FALSE
	* @return ERR_SUCC  
	*/
	errno_t drag_mode_enable(BOOL enable);

	/**
	* @brief 
	* @param in_drag 
	* @return ERR_SUCC  
	*/
	errno_t is_in_drag_mode(BOOL *in_drag);

	/**
	* @brief 
	* @param state 
	* @return ERR_SUCC  
	*/
	errno_t get_robot_state(RobotState *state);

	/**
	* @brief Set
	* @param tcp_position TCP position
	* @return ERR_SUCC  
	*/
	errno_t get_tcp_position(CartesianPose *tcp_position);

	/**
	* @brief 
	* @param joint_position 
	* @return ERR_SUCC  
	*/
	errno_t get_joint_position(JointValue *joint_position);

	/**
	* @brief 
	* @param in_collision 
	* @return ERR_SUCC  
	*/
	errno_t is_in_collision(BOOL *in_collision);

	/**
	* @brief 
	* @param on_limit 
	* @return ERR_SUCC  
	*/
	errno_t is_on_limit(BOOL *on_limit);

	/**
	* @brief 
	* @param in_pos 
	* @return ERR_SUCC  
	*/
	errno_t is_in_pos(BOOL *in_pos);

	/**
	 * @brief Setinpos,0.003rad
	 * @param handle 
	 * @param thresholding ,in_posReturns1
	 * @param ERR_SUCC 
	 */
	errno_t set_in_pos_thresholding(const double thresholding);

	/**
	 * @brief inpos,0.003rad
	 * @param handle 
	 * @param thresholding 
	 * @param ERR_SUCC 
	 */
	errno_t get_in_pos_thresholding(double *thresholding);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t collision_recover();

	/**
	* @brief  
	* @return ERR_SUCC  
	*/
	errno_t clear_error();

	/**
	* @brief Set
	* @param level  ,0-5 ,0,125N,250N,375N,4100N,5125N
	* @return ERR_SUCC  
	*/
	errno_t set_collision_level(const int level);

	/**
	* @brief Set
	* @return ERR_SUCC  
	*/
	errno_t get_collision_level(int *level);

	/**
	* @brief ,Set
	* @param ref_pos 
	* @param cartesian_pose 
	* @param joint_pos 
	* @return ERR_SUCC  
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief joint position,Set
	* @param joint_pos 
	* @param cartesian_pose 
	* @return ERR_SUCC  
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose);

	/**
	* @brief 
	* @param rpy 
	* @param rot_matrix 
	* @return ERR_SUCC  
	*/
	errno_t rpy_to_rot_matrix(const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief 
	* @param rot_matrix 
	* @param rpy RPY
	* @return ERR_SUCC  
	*/
	errno_t rot_matrix_to_rpy(const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief 
	* @param quaternion 
	* @param rot_matrix 
	* @return ERR_SUCC  
	*/
	errno_t quaternion_to_rot_matrix(const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief 
	* @param rot_matrix 
	* @param quaternion 
	* @return ERR_SUCC  
	*/
	errno_t rot_matrix_to_quaternion(const RotMatrix *rot_matrix, Quaternion *quaternion);

	/**
	* @brief 
	* @param func 
	* @param error_code 
	*/
	errno_t set_error_handler(CallBackFuncType func);

	/**
	* @brief Set
	* @param payload ,
	* @return ERR_SUCC  
	*/
	errno_t set_payload(const PayLoad *payload);

	/**
	* @brief 
	* @param payload 
	* @return ERR_SUCC  
	*/
	errno_t get_payload(PayLoad *payload);

	/**
	* @brief SDK
	* @param version SDK
	* @return ERR_SUCC  
	*/
	errno_t get_sdk_version(char *version);

	/**
	* @brief IP
	* @param controller_name 
	* @param ip_list ip,ReturnsIP,,ReturnsIP
	* @return ERR_SUCC  
	*/
	errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief 
	* @param status 
	* @return ERR_SUCC  
	*/
	errno_t get_robot_status(RobotStatus *status);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t motion_abort();

	/**
	* @brief Set,get_last_errorSet,get_last_error,Set
	* @return ERR_SUCC  
	*/
	errno_t set_errorcode_file_path(char *path);

	/**
	* @brief ,clear_error,
	* @return ERR_SUCC  
	*/
	errno_t get_last_error(ErrorCode *code);

	/**
	* @brief Set,TRUE,,OutputPrint debug information,FALSE,Print debug information
	* @return ERR_SUCC  
	*/
	errno_t set_debug_mode(BOOL mode);

	/**
	* @brief SetParameters
	* @param para Parameters
	* @return ERR_SUCC  
	*/
	errno_t set_traj_config(const TrajTrackPara *para);

	/**
	* @brief Parameters
	* @param para Parameters
	* @return ERR_SUCC  
	*/
	errno_t get_traj_config(TrajTrackPara *para);

	/**
	* @brief 
	* @param mode TRUE,,FALSE,
	* @param filename ,filename,
	* @return ERR_SUCC  
	*/
	errno_t set_traj_sample_mode(const BOOL mode, char *filename);

	/**
	* @brief 
	* @param mode TRUE,,FALSE,,
	* @return ERR_SUCC  
	*/
	errno_t get_traj_sample_status(BOOL *sample_status);

	/**
	* @brief 
	* @param filename 
	* @return ERR_SUCC  
	*/
	errno_t get_exist_traj_file_name(MultStrStorType *filename);

	/**
	* @brief 
	* @param src 
	* @param dest ,100,,
	* @return ERR_SUCC  
	*/
	errno_t rename_traj_file_name(const char *src, const char *dest);

	/**
	* @brief 
	* @param filename ,
	* @return ERR_SUCC  
	*/
	errno_t remove_traj_file(const char *filename);

	/**
	* @brief 
	* @param filename ,,
	* @return ERR_SUCC  
	*/
	errno_t generate_traj_exe_file(const char *filename);

	/**
	* @brief SERVO,SERVOSet,SERVOSet
	* @return ERR_SUCC  
	*/
	errno_t servo_move_use_none_filter();

	/**
	* @brief SERVO,SERVOSet,SERVOSet
	* @param cutoffFreq 
	* @return ERR_SUCC  
	*/
	errno_t servo_move_use_joint_LPF(double cutoffFreq);

	/**
	* @brief SERVO,SERVOSet,SERVOSet
	* @param max_vr ()deg/s
	* @param max_ar ()deg/s^2
	* @param max_jr ()deg/s^3
	* @return ERR_SUCC  
	*/
	errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO,SERVOSet,SERVOSet
	* @param max_vp ().:mm/s
	* @param max_ap ().:mm/s^2
	* @param max_jp ():mm/s^3
	* @param max_vr ()deg/s
	* @param max_ar ()deg/s^2
	* @param max_jr ()deg/s^3
	* @return ERR_SUCC  
	*/
	errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO,SERVOSet,SERVOSet
	* @param max_buf 
	* @param kp 
	* @param kv 
	* @param ka 
	* @return ERR_SUCC  
	*/
	errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka);

	/**
	* @brief SERVOParametersSet
	* @param max_buf 
	* @param kp 
	* @return ERR_SUCC  
	*/
	errno_t servo_speed_foresight(int max_buf, double kp);

	/**
	* @brief SetSDK
	* @param filepath SDK
	* @return ERR_SUCC  
	*/
	errno_t set_SDK_filepath(const char *filepath);

	/**
	* @brief set_SDK_filepath
	*/
	static errno_t static_Set_SDK_filepath(const char *filepath);

	/**
	* @brief Set
	* @param sensor_brand ,1,2,3 
	* @return ERR_SUCC  
	*/
	errno_t set_torsenosr_brand(int sensor_brand);

	/**
	* @brief 
	* @param sensor_brand ,1,2,3 
	* @return ERR_SUCC  
	*/
	errno_t get_torsenosr_brand(int *sensor_brand);

	/**
	* @brief 
	* @param sensor_mode 0,1
	* @return ERR_SUCC  
	*/
	errno_t set_torque_sensor_mode(int sensor_mode);

	/**
	* @brief SetParameters
	* @param axis ,0~5
	* @param opt , 1 2 3 4 5 6 fx fy fz mx my mz 0
	* @param ftUser ,
	* @param ftConstant ,Set0
	* @param ftNnormalTrack ,Set0,
	* @param ftReboundFK ,
	* @return ERR_SUCC  
	*/
	errno_t set_admit_ctrl_config(int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);

	/**
	* @brief 
	* @param joint_pos 
	* @return ERR_SUCC  
	*/
	errno_t start_torq_sensor_payload_identify(const JointValue *joint_pos);

	/**
	* @brief 
	* @param identify_status 0,1,2
	* @return ERR_SUCC  
	*/
	errno_t get_torq_sensor_identify_staus(int *identify_status);

	/**
	* @brief 
	* @param payload 
	* @return ERR_SUCC  
	*/
	errno_t get_torq_sensor_payload_identify_result(PayLoad *payload);

	/**
	* @brief Set
	* @param payload 
	* @return ERR_SUCC  
	*/
	errno_t set_torq_sensor_tool_payload(const PayLoad *payload);

	/**
	* @brief 
	* @param payload 
	* @return ERR_SUCC  
	*/
	errno_t get_torq_sensor_tool_payload(PayLoad *payload);

	/**
	* @brief 
	* @param enable_flag 0,1
	* @return ERR_SUCC  
	*/
	errno_t enable_admittance_ctrl(const int enable_flag);

	/**
	* @brief Setinitialization state
	* @param sensor_compensation ,1,0
	* @param compliance_type 0  1 ,2 
	* @return ERR_SUCC  
	*/
	errno_t set_compliant_type(int sensor_compensation, int compliance_type);

	/**
	* @brief initialization state
	* @param sensor_compensation ,1,0
	* @param compliance_type 0  1 ,2 
	* @return ERR_SUCC  
	*/
	errno_t get_compliant_type(int *sensor_compensation, int *compliance_type);

	/**
	* @brief Parameters
	* @param admit_ctrl_cfg Parameters
	* @return ERR_SUCC  
	*/
	errno_t get_admit_ctrl_config(RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief Setip
	* @param type 0tcp/ip,1RS485
	* @param ip_addr
	* @param porttcp/ip
	* @return ERR_SUCC  
	*/
	errno_t set_torque_sensor_comm(const int type, const char *ip_addr, const int port);

	/**
	* @brief ip
	* @param type 0tcp/ip,1RS485
	* @param ip_addr
	* @param porttcp/ip
	* @return ERR_SUCC  
	*/
	errno_t get_torque_sensor_comm(int *type, char *ip_addr, int *port);

	/**
	* @brief Set
	* @param torque_sensor_filter ,:Hz
	*/
	errno_t set_torque_sensor_filter(const float torque_sensor_filter);

	/**
	* @brief 
	* @param torque_sensor_filter ,:Hz
	*/
	errno_t get_torque_sensor_filter(float *torque_sensor_filter);

	/**
	* @brief SetParameters
	* @param torque_sensor_soft_limit Parameters
	*/
	errno_t set_torque_sensor_soft_limit(const FTxyz torque_sensor_soft_limit);

	/**
	* @brief Parameters
	* @param torque_sensor_soft_limit Parameters
	*/
	errno_t get_torque_sensor_soft_limit(FTxyz *torque_sensor_soft_limit);

	/**
	* @brief 
	* @return ERR_SUCC  
	*/
	errno_t disable_force_control();

	/**
	* @brief SetParameters
	* @param vel_cfgParameters
	* @return ERR_SUCC  
	*/
	errno_t set_vel_compliant_ctrl(const VelCom *vel_cfg);

	/**
	* @brief Set
	* @param ft
	* @return ERR_SUCC  
	*/
	errno_t set_compliance_condition(const FTxyz *ft);

	/**
	* @brief Setexception,SDK
	* @param millisecond Parameters,
	* @param mnt exception
	* @return ERR_SUCC  
	*/
	errno_t set_network_exception_handle(float millisecond, ProcessType mnt);

	/**
	* @brief Set
	* @param millisecond Parameters,
	* @return ERR_SUCC  
	*/
	errno_t set_status_data_update_time_interval(float millisecond);

	/**
	* @brief Set
	* @param seconds Parameters,
	* @return ERR_SUCC  
	*/
	errno_t set_block_wait_timeout(float seconds);

	/**
	* @brief Set
	* @param ftFrame 0 1 
	* @return ERR_SUCC  
	*/
	errno_t set_ft_ctrl_frame(const int ftFrame);

	/**
	* @brief 
	* @param ftFrame 0 1
	* @return ERR_SUCC  
	*/
	errno_t get_ft_ctrl_frame(int* ftFrame);

	/**
	* @brief dhParameters
	* @param dh_param DHParameters
	* @return ERR_SUCC ,
	*/
	errno_t get_dh_param(DHParam* dh_param);

	/**
	* @brief Set
	* @param angleX X
	* @param angleZ Z
	* @return ERR_SUCC  
	*/
	errno_t set_installation_angle(double angleX, double angleZ);

	/**
	* @brief 
	* @param quat 
	* @param appang RPY
	* @return ERR_SUCC  
	*/
	errno_t get_installation_angle(Quaternion* quat, Rpy* appang);

	/**
	* @brief SettioV3Parameters
	* @param vout_enable ,0:,1
	* @param vout_vol  0:24v 1:12v
	* @return ERR_SUCC  
	*/
	errno_t set_tio_vout_param(int vout_enable, int vout_vol);

	/**
	* @brief tioV3Parameters
	* @param vout_enable ,0:,1
	* @param vout_vol  0:24v 1:12v
	* @return ERR_SUCC  
	*/
	errno_t get_tio_vout_param(int* vout_enable, int* vout_vol);

	/**
	* @brief modified
	* @param sign_info Parameters
	* @return ERR_SUCC  
	*/
	errno_t add_tio_rs_signal(SignInfo sign_info);

	/**
	* @brief 
	* @param sig_name 
	* @return ERR_SUCC  
	*/
	errno_t del_tio_rs_signal(const char* sig_name);

	/**
	* @brief RS485
	* @param chn_id 
	* @param data 
	* @return ERR_SUCC  
	*/
	errno_t send_tio_rs_command(int chn_id, uint8_t* data,int buffsize);

	/**
	* @brief 
	* @param SignInfo* 
	* @return ERR_SUCC  
	*/
	errno_t get_rs485_signal_info(SignInfo* sign_info_array, int* array_len);

	/**
	* @brief Settio
	* @param pin_type tio 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tioDI Pins: 0:0x00 DI2NPN,DI1NPN,1:0x01 DI2NPN,DI1PNP, 2:0x10 DI2PNP,DI1NPN,3:0x11 DI2PNP,DI1PNP
							 DO Pins: 84DO2,DO1,0x0 DONPNOutput, 0x1 DOPNPOutput, 0x2 DOOutput, 0xF RS485H
							 AI Pins: 0:,RS485L, 1:RS485L,
	* @return ERR_SUCC  
	*/
	errno_t set_tio_pin_mode(int pin_type, int pin_mode);

	/**
	* @brief tio
	* @param pin_type tio 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tioDI Pins: 0:0x00 DI2NPN,DI1NPN,1:0x01 DI2NPN,DI1PNP, 2:0x10 DI2PNP,DI1NPN,3:0x11 DI2PNP,DI1PNP
							 DO Pins: 84DO2,DO1,0x0 DONPNOutput, 0x1 DOPNPOutput, 0x2 DOOutput, 0xF RS485H
							 AI Pins: 0:,RS485L, 1:RS485L,
	* @return ERR_SUCC  
	*/
	errno_t get_tio_pin_mode(int pin_type, int* pin_mode);

	/**
	* @brief RS485Parameters
	* @param ModRtuComm SetModbus RTU,ModbusID
	* @return ERR_SUCC  
	*/
	errno_t set_rs485_chn_comm(ModRtuComm mod_rtu_com);

	/**
	* @brief RS485Parameters
	* @param ModRtuComm chn_idParameters
	* @return ERR_SUCC  
	*/
	errno_t get_rs485_chn_comm(ModRtuComm* mod_rtu_com);

	/**
	* @brief RS485
	* @param chn_id 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode 0: Modbus RTU, 1: Raw RS485, 2, torque sensor
	* @return ERR_SUCC  
	*/
	errno_t set_rs485_chn_mode(int chn_id, int chn_mode);

	/**
	* @brief RS485
	* @param chn_id Parameters 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode OutputParameters  0: Modbus RTU, 1: Raw RS485, 2, torque sensor
	* @return ERR_SUCC  
	*/
	errno_t get_rs485_chn_mode(int chn_id, int* chn_mode);

	/**
	* @brief ftp
	* @return ERR_SUCC  
	*/
	errno_t init_ftp_client();

	/**
	* @brief ftp(app)
	* @param password 
	* @return ERR_SUCC  
	*/
	errno_t init_ftp_client_with_ssl(char* password);

	/**
	* @brief ftp
	* @return ERR_SUCC  
	*/
	errno_t close_ftp_client();
	/**
	* @brief 
	* @param remote 
	* @param local 
	* @param opt 1 2
	* @return ERR_SUCC  
	*/
	errno_t download_file(char* local, char* remote, int opt);

	/**
	* @brief 
	* @param remote 
	* @param local 
	* @param opt 1 2
	* @return ERR_SUCC  
	*/
	errno_t upload_file(char* local, char* remote, int opt);


	/**
	* @brief 
	* @param remote 
	* @param opt 1 2
	* @return ERR_SUCC  
	*/
	errno_t del_ftp_file(char* remote, int opt);

	/**
	* @brief 
	* @param remote 
	* @param des 
	* @param opt 1 2
	* @return ERR_SUCC  
	*/
	errno_t rename_ftp_file(char* remote, char* des, int opt);

	/**
	* @brief 
	* @param remotedir 
	* @param type 0 1 2
	* @param ret 
	* @return ERR_SUCC  
	*/
	errno_t get_ftp_dir(const char* remotedir, int type, char* ret);

	~JAKAZuRobot();

	

private:
	class BIFClass;
	BIFClass *ptr;
};


#undef DLLEXPORT_API
#endif
