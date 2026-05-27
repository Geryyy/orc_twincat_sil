#pragma once

#include"TcDef.h"
#include"TcRtInterfaces.h"
#include"Axis.h"
#include"AxisControl.h"
#include"AxisSensor.h"
#include"KsoeSlave.h"
#include"KsoeSlaveParameters.h"
#include"AdsR0.h"
#include"FsoeSlave.h"
#include"AxisLimits.h"
#include"Motorcontroller.h"
#include"MotorcontrollerParameters.h"


#define ROBOT_NUM_AXIS 9
#define ROBOT_NUM_SLAVE 4
#define ROBOT_NUM_MOTORCONTROLLER 2
#define ROBOT_RESET_TICKS (5000000/125) //Ticks
#define ROBOT_SOFT_STOP_TICKS (500000/125) //Ticks
#define OUTPUT_COOLDOWN_TIME 1000*8 //Ticks
//Once the workspace limit is reached, the robot may be enabled again and only
//velocity away from this limit is allowed. To account for sensor noise, a
//tolerance band is defined additionally, which allows minimal movements further
//into the limit. This tolerance band is defined by ROBOT_VELOCITY_TOLERANCE.
#define ROBOT_VELOCITY_TOLERANCE 0.01 //m/s
#include"RobotParameters.h"

struct RobotParameters;

class Robot : public CAdsR0FifoPort {
  private:
    ITcRTimeTaskPtr task;
    CTcTrace *trace;
    KsoeSlave *slave[ROBOT_NUM_SLAVE];
    FsoeSlave *mft;
	Motorcontroller *motorcontroller[ROBOT_NUM_MOTORCONTROLLER];
    ITComObjectServer *com_object_server;
    RobotParameters *parameters;
    KsoeState last_collective_state;
    EventHandler* event_handler;

    DWORD temperature_request_counter;
    DWORD reset_state_counter;
    DWORD soft_stop_counter;
    AxisLimits *limits;

    bool checkLastErrorAndReset();
    bool hasWarning();
    bool hasValidInput();
	void gravitationVector9DOF(double *q, double *g);
    void gravitationVector7DOF(double *q, double *g);
	void endeffectorVelocity9DOF(double *q, double *qp, double *cartesian_velocity);
    void endeffectorVelocity7DOF(double *q, double *qp, double *cartesian_velocity);
    void endeffectorPose9DOF(double *q, double *position, double *orientation);
    void endeffectorPose7DOF(double *q, double *position, double *orientation);

	int disableMotion_output_timer;
	bool outside_workspace_error;
	bool outside_workspace_acknowledge;
	double outside_workspace_cartesian_position[3];

  public:
    Robot(RobotParameters *parameters, AxisLimits *limits, ITComObjectServer *com_object_server, CTcTrace *trace);
    ~Robot();
    KsoeState getCollectiveState();
    bool isMotionEnabled();
    void enableMotion();
    void performSoftStop();
    void disableMotion();
    Event* getEvent();
    void ReadSensor(AxisSensor *input, MFTSensor *input_mft);
    void WriteControl(AxisSensor *input, AxisControl *output, MotorcontrollerModeOfOperation mode_of_operation, DWORD dc_time_L);
	MotorcontrollerModeOfOperation getCollectiveModeOfOperation();

	bool CheckWorkspaceLimits(double *q, double *qp, double *cartesian_position, double *cartesian_orientation);

    //CAds
    virtual void AdsReadCon(AmsAddr& rAddr, ULONG invokeId, ULONG result, ULONG cbLength, PVOID pData);
};
