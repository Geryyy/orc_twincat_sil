///////////////////////////////////////////////////////////////////////////////
// Robot.cpp
#include "TcPch.h"
#pragma hdrstop

#include "Robot.h"

Robot::Robot(RobotParameters *parameters, AxisLimits *limits, ITComObjectServer *com_object_server, CTcTrace *trace) {
  this->com_object_server=com_object_server;
  this->trace=trace;
  this->parameters=parameters;
  this->reset_state_counter=ROBOT_RESET_TICKS;
  this->limits=limits;
  soft_stop_counter=0;
  event_handler = new EventHandler(trace);

  slave[0]=new KsoeSlave(trace, event_handler, parameters->ksoe_slave_parameters[0].connection_id, parameters->ksoe_slave_parameters[0].input_buffer,
    parameters->ksoe_slave_parameters[0].output_buffer,&parameters->ksoe_slave_parameters[0].pds_parameters,limits,&parameters->axis[0 + ROBOT_NUM_MOTORCONTROLLER]);
  slave[1]=new KsoeSlave(trace, event_handler, parameters->ksoe_slave_parameters[1].connection_id, parameters->ksoe_slave_parameters[1].input_buffer,
    parameters->ksoe_slave_parameters[1].output_buffer,&parameters->ksoe_slave_parameters[1].pds_parameters,limits,&parameters->axis[1 + ROBOT_NUM_MOTORCONTROLLER],&parameters->axis[2 + ROBOT_NUM_MOTORCONTROLLER]);
  slave[2]=new KsoeSlave(trace, event_handler, parameters->ksoe_slave_parameters[2].connection_id, parameters->ksoe_slave_parameters[2].input_buffer,
    parameters->ksoe_slave_parameters[2].output_buffer,&parameters->ksoe_slave_parameters[2].pds_parameters,limits,&parameters->axis[3 + ROBOT_NUM_MOTORCONTROLLER],&parameters->axis[4 + ROBOT_NUM_MOTORCONTROLLER]);
  slave[3]=new KsoeSlave(trace, event_handler, parameters->ksoe_slave_parameters[3].connection_id, parameters->ksoe_slave_parameters[3].input_buffer,
    parameters->ksoe_slave_parameters[3].output_buffer,&parameters->ksoe_slave_parameters[3].pds_parameters,limits,&parameters->axis[5 + ROBOT_NUM_MOTORCONTROLLER],&parameters->axis[6 + ROBOT_NUM_MOTORCONTROLLER]);
 
  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
	  motorcontroller[i] = new Motorcontroller(trace, event_handler, parameters->motorcontroller_parameters[i].motorcontroller_id, parameters->motorcontroller_parameters[i].input_buffer,
		  parameters->motorcontroller_parameters[i].output_buffer, limits, &parameters->axis[i]);
  }

  if(parameters->fsoe_slave_parameters.input_buffer!=NULL &&
    parameters->fsoe_slave_parameters.output_buffer!=NULL &&
    parameters->fsoe_slave_parameters.connection_id!=0 &&
    parameters->fsoe_slave_parameters.slave_address!=0) {
    mft=new FsoeSlave(trace, event_handler, parameters->fsoe_slave_parameters.connection_id, parameters->fsoe_slave_parameters.slave_address,
      (MFT_FSoE_Inputs*)parameters->fsoe_slave_parameters.input_buffer, (MFT_FSoE_Outputs*)parameters->fsoe_slave_parameters.output_buffer,
      parameters->NSR_INDICATOR,parameters->NSR_INPUT);
  }
  else {
    mft=NULL;
    trace->Log(tlInfo, "%s Robot::Robot(): Robot created without MFT.", parameters->name);
  }
  InitAmsPort(com_object_server, 2000);
  temperature_request_counter= ULONG_MAX;
  trace->Log(tlInfo,"%s Robot::Robot(): Robot created successfully.", parameters->name);
  last_collective_state=KsoeState::NONE;

  disableMotion_output_timer = 0;
  outside_workspace_error = false;
  outside_workspace_acknowledge = false;
  memset(outside_workspace_cartesian_position, 0, 3 * sizeof(double));
}

Robot::~Robot() {
  for(int i = 0;i<ROBOT_NUM_SLAVE;i++) {
    safe_delete(slave[i]);
  }
  safe_delete(mft);
  safe_delete(event_handler);
  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
	  safe_delete(motorcontroller[i]);
  }
  ShutdownAmsPort();
  trace->Log(tlInfo,"%s Robot::Robot(): Robot deleted.", parameters->name);
}

void Robot::ReadSensor(AxisSensor *input, MFTSensor *input_mft) {
  // handle pending ADS indications and confirmations
  CheckOrders();
  
  if(reset_state_counter!=0) {
    reset_state_counter--;
  }
  //Do not operate as long as the input from the slaves is not yet valid.
  if(hasValidInput() && !reset_state_counter) {
    temperature_request_counter++;
    //Request the temperature of all slaves every 5 seconds (at a cycle time of 125 �s)
    //ADS Request should work as soon as the slave is in state OP.
    if(temperature_request_counter>=5000000/125) {
      temperature_request_counter=0;
      DWORD nOffset=(0x2200<<16)|(0x000a);
      AmsAddr addr;
      addr.netId=parameters->netid;
      for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
        addr.port=parameters->ksoe_slave_parameters[i].slave_address;
        //Use slave address as request number to later store the received data.
        AdsReadReq(addr, addr.port, ADSIGRP_CANOPEN_SDO, nOffset, 2);
      }
    }

    //Only proceed if there are no error states present.
    if(!checkLastErrorAndReset()) {
      slave[0]->checkCrcReadSensor(&input[ROBOT_NUM_MOTORCONTROLLER], NULL);
      slave[1]->checkCrcReadSensor(&input[1+ ROBOT_NUM_MOTORCONTROLLER], &input[2+ ROBOT_NUM_MOTORCONTROLLER]);
      slave[2]->checkCrcReadSensor(&input[3+ ROBOT_NUM_MOTORCONTROLLER], &input[4+ ROBOT_NUM_MOTORCONTROLLER]);
      slave[3]->checkCrcReadSensor(&input[5+ ROBOT_NUM_MOTORCONTROLLER], &input[6+ ROBOT_NUM_MOTORCONTROLLER]);

	  if (limits->is9DOF) {
		  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
			  motorcontroller[i]->readSensor(&input[i]);
		  }
	  }

      checkLastErrorAndReset();
    }
  }
  else {
    for(int i=0;i<ROBOT_NUM_AXIS;i++) {
      input[i].motor_torque_feedback=0;
      input[i].position=0;
      input[i].torque=0;

	  input[i].force = 0;
	  input[i].position_linear = 0;
	  input[i].position_linear_filtered = 0;
	  input[i].velocity_linear_filtered = 0;
    }
  }
  if(mft!=NULL && input_mft!=NULL) {
    if(!reset_state_counter) {
      mft->ReadSensor(input_mft);
    }
  }

  KsoeState collective_state=getCollectiveState();
  if(collective_state!=last_collective_state &&
    collective_state!=KsoeState::NONE) {
    trace->Log(tlVerbose, "%s Robot::CycleUpdate(): Collective state %s (0x%02x) -> %s (0x%02x).",
      parameters->name,toString(last_collective_state),last_collective_state,toString(collective_state),collective_state);
    if(collective_state==KsoeState::PROCESS_DATA) {
      trace->Log(tlInfo, "%s Robot::CycleUpdate(): Robot is now fully initialized.",parameters->name);
    }
    last_collective_state=collective_state;
  }
}

void Robot::WriteControl(AxisSensor *input, AxisControl *output, MotorcontrollerModeOfOperation mode_of_operation, DWORD dc_time_L) {
  bool axes_enabled;
  //Do not operate as long as the input from the slaves is not yet valid.
  //Only proceed if there are no error states present.
  if(hasValidInput() && !reset_state_counter && !checkLastErrorAndReset()) {
    if(soft_stop_counter==0 && isMotionEnabled() && hasWarning()) {
      soft_stop_counter=ROBOT_SOFT_STOP_TICKS;
      if(mft!=NULL) {
        mft->setLEDState(LEDState::RED);
      }
    }
    if(soft_stop_counter!=0) {
		if (limits->is9DOF) {
			double q[ROBOT_NUM_AXIS];
			double g[ROBOT_NUM_AXIS];
			for (int i = 0; i < ROBOT_NUM_AXIS; i++) {
				q[i] = input[i].position;
			}
			gravitationVector9DOF(q, g);
			for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
				output[i].motor_force = g[i];
			}
			for (int i = ROBOT_NUM_MOTORCONTROLLER; i < ROBOT_NUM_AXIS; i++) {
				output[i].motor_torque = g[i];
			}
		}
		else {
			double q[ROBOT_NUM_AXIS-ROBOT_NUM_MOTORCONTROLLER];
			double g[ROBOT_NUM_AXIS-ROBOT_NUM_MOTORCONTROLLER];
			for (int i = 0; i<ROBOT_NUM_AXIS-ROBOT_NUM_MOTORCONTROLLER; i++) {
				q[i] = input[i + ROBOT_NUM_MOTORCONTROLLER].position;
			}
			gravitationVector7DOF(q, g);
			for (int i = 0; i < ROBOT_NUM_AXIS - ROBOT_NUM_MOTORCONTROLLER; i++) {
				output[i + ROBOT_NUM_MOTORCONTROLLER].motor_torque = g[i];
			}
		}
      //Perform a soft stop by using a P velocity controller
	  if (limits->is9DOF) {
		  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
			  output[i].motor_velocity = 0.0;
			  motorcontroller[i]->performSoftStop(&input[i], &output[i], dc_time_L);
		  }
	  }
      slave[0]->performSoftStop(&input[0 + ROBOT_NUM_MOTORCONTROLLER], NULL, &output[0 + ROBOT_NUM_MOTORCONTROLLER], NULL, dc_time_L);
      slave[1]->performSoftStop(&input[1 + ROBOT_NUM_MOTORCONTROLLER], &input[2 + ROBOT_NUM_MOTORCONTROLLER], &output[1 + ROBOT_NUM_MOTORCONTROLLER], &output[2 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);
      slave[2]->performSoftStop(&input[3 + ROBOT_NUM_MOTORCONTROLLER], &input[4 + ROBOT_NUM_MOTORCONTROLLER], &output[3 + ROBOT_NUM_MOTORCONTROLLER], &output[4 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);
      slave[3]->performSoftStop(&input[5 + ROBOT_NUM_MOTORCONTROLLER], &input[6 + ROBOT_NUM_MOTORCONTROLLER], &output[5 + ROBOT_NUM_MOTORCONTROLLER], &output[6 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);
      //Check if soft stop is completed on all axes. If it is completed,
      //the corresponding motor_enable signal is false.
      axes_enabled=false;
      for(int i=0;i<ROBOT_NUM_AXIS;i++) {
        axes_enabled|=output[i].motor_enable;
      }
      //If any axis is still enabled, continue with soft stop. Otherwise, disable
      //motion completely to end the soft stop phase. This also clears the warning.
      if(axes_enabled) {
        soft_stop_counter--;
        if(soft_stop_counter==0) {
          trace->Log(tlWarning, "%s Robot::WriteControl(): Soft stop time exceeded, disabling motion.", parameters->name);
          disableMotion();
        }
      }
      else {
        soft_stop_counter=0;
        trace->Log(tlInfo, "%s Robot::WriteControl(): Soft stop completed.", parameters->name);
        disableMotion();
      }
    }
    else {
      slave[0]->statemachineWriteControlGenerateCrc(&output[0 + ROBOT_NUM_MOTORCONTROLLER], NULL, dc_time_L);
      slave[1]->statemachineWriteControlGenerateCrc(&output[1 + ROBOT_NUM_MOTORCONTROLLER], &output[2 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);
      slave[2]->statemachineWriteControlGenerateCrc(&output[3 + ROBOT_NUM_MOTORCONTROLLER], &output[4 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);
      slave[3]->statemachineWriteControlGenerateCrc(&output[5 + ROBOT_NUM_MOTORCONTROLLER], &output[6 + ROBOT_NUM_MOTORCONTROLLER], dc_time_L);

	  if (limits->is9DOF) {
		  //Handle mode of operation
		  if (getCollectiveModeOfOperation() != mode_of_operation) {
			  if (!isMotionEnabled()) {
				  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
					  motorcontroller[i]->setModeOfOperation(mode_of_operation);
				  }
			  }
			  else {
				  performSoftStop();
			  }
		  }
		  //Handle motorcontroller state machine and write outputs
		  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
			  motorcontroller[i]->statemachineWriteControl(&output[i], dc_time_L);
		  }
	  }
    }
    checkLastErrorAndReset();
  }

  // decrement cooldown timer
  if (disableMotion_output_timer > 0) {
	  disableMotion_output_timer--;
  }
  else {
	  disableMotion_output_timer = 0;
  }

  // decrement all slave cooldown timers
  slave[0]->cooldownTimer();
  slave[1]->cooldownTimer();
  slave[2]->cooldownTimer();
  slave[3]->cooldownTimer();
  if (limits->is9DOF) {
	  for (int i = 0; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
		  motorcontroller[i]->cooldownTimer();
	  }
  }
}

bool Robot::CheckWorkspaceLimits(double *q, double *qp, double *cartesian_position, double *cartesian_orientation) {
	double cartesian_velocity[3];
	// get cartesian position and velocity
	if (limits->is9DOF) {
		endeffectorPose9DOF(q, cartesian_position, cartesian_orientation);
		endeffectorVelocity9DOF(q, qp, cartesian_velocity);
	}
	else {
		//robot joint positions are q[2] .. q[8]
		endeffectorPose7DOF(&q[ROBOT_NUM_MOTORCONTROLLER], cartesian_position, cartesian_orientation);
		endeffectorVelocity7DOF(&q[ROBOT_NUM_MOTORCONTROLLER], &qp[ROBOT_NUM_MOTORCONTROLLER], cartesian_velocity);
	}
	memcpy(outside_workspace_cartesian_position, cartesian_position, 3 * sizeof(double));

	// define error flag for each direction
	outside_workspace_error = (cartesian_position[0]<limits->workspace_limits[0] && cartesian_velocity[0]<-ROBOT_VELOCITY_TOLERANCE) ||
		(cartesian_position[0]>limits->workspace_limits[1] && cartesian_velocity[0]>ROBOT_VELOCITY_TOLERANCE) ||
		(cartesian_position[1]<limits->workspace_limits[2] && cartesian_velocity[1]<-ROBOT_VELOCITY_TOLERANCE) ||
		(cartesian_position[1]>limits->workspace_limits[3] && cartesian_velocity[1]>ROBOT_VELOCITY_TOLERANCE)||
		(cartesian_position[2]<limits->workspace_limits[4] && cartesian_velocity[2]<-ROBOT_VELOCITY_TOLERANCE) || 
		(cartesian_position[2]>limits->workspace_limits[5] && cartesian_velocity[2]>ROBOT_VELOCITY_TOLERANCE);

	// check for error conditions
	if(outside_workspace_error && !outside_workspace_acknowledge) {
		trace->Log(tlWarning, "Robot::CheckWorkspaceLimits(): Robot outside workspace.\n Current position: [%.2f, %.2f, %.2f].\n Limits: x: [%.2f, %.2f], y: [%.2f, %.2f], z: [%.2f, %.2f].",
			cartesian_position[0], cartesian_position[1], cartesian_position[2],
			limits->workspace_limits[0], limits->workspace_limits[1], limits->workspace_limits[2],
			limits->workspace_limits[3], limits->workspace_limits[4], limits->workspace_limits[5]);
		performSoftStop();
		event_handler->set(0, 0, EventCode::RobotError_WORKSPACE_LIMIT, EventType::ERROR);
	}
	// check if error is cleared
	else if(!outside_workspace_error && outside_workspace_acknowledge) {
		trace->Log(tlError, "Robot::CheckWorkspaceLimits(): Robot outside workspace cleared.");
	} 
  
	outside_workspace_acknowledge = outside_workspace_error;

	return cartesian_position[0] < limits->workspace_limits[0] ||
		cartesian_position[0] > limits->workspace_limits[1] ||
		cartesian_position[1] < limits->workspace_limits[2] ||
		cartesian_position[1] > limits->workspace_limits[3] ||
		cartesian_position[2] < limits->workspace_limits[4] ||
		cartesian_position[2] > limits->workspace_limits[5];
}

bool Robot::checkLastErrorAndReset() {
  bool reset=false;
  bool emergency=false;
  for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
	  EventCode error=slave[i]->getLastError();
    if(error!=EventCode::NONE) {
      trace->Log(tlError,"%s Robot::checkLastErrorAndReset(): Reset due to error %s (%d) of slave %d.",
        parameters->name,toString(error),(int)error,slave[i]->getConnectionID());
      reset=true;
    }
    error=slave[i]->getMotionError();
    if(error!=EventCode::NONE) {
      if(slave[i]->getMotionStateTarget()!=MotionState::STOP) {
        trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted due to error %s (%d) of slave %d.",
          parameters->name, toString(error), (int)error, slave[i]->getConnectionID());
        slave[i]->disableMotion();
      }
      emergency=true;
    }
    error=slave[i]->getSensorError();
    if(error!=EventCode::NONE) {
      if(slave[i]->getMotionStateTarget()!=MotionState::STOP) {
        trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted due to sensor error %s (%d) of slave %d.",
          parameters->name, toString(error), (int)error, slave[i]->getConnectionID());
        slave[i]->disableMotion();
      }
      emergency=true;
    }
  }
  if (mft != NULL) {
	  EventCode error = mft->getLastError();
	  if (error != EventCode::NONE) {
		  trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Reset due to error %s (%d) of slave MFT.",
			  parameters->name, toString(error), (int)error);
		  reset = true;
	  }
	  error = mft->getSensorError();
	  if (error != EventCode::NONE) {
		  trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted due to sensor error %s (%d) of slave MFT.",
			  parameters->name, toString(error), (int)error);
		  emergency = true;
	  }
  }
  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
	  EventCode error = motorcontroller[i]->getLastError();
	  if (error != EventCode::NONE) {
		  trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Reset due to error %s (%d) of motorcontroller %d.",
			  parameters->name, toString(error), (int)error, motorcontroller[i]->getMotorcontrollerID());
		  reset = true;
	  }
	  error = motorcontroller[i]->getMotionError();
	  if (error != EventCode::NONE) {
		  if (motorcontroller[i]->getMotionStateTarget() != MotionState::STOP) {
			  trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted due to error %s (%d) of motorcontroller %d.",
				  parameters->name, toString(error), (int)error, motorcontroller[i]->getMotorcontrollerID());
			  motorcontroller[i]->disableMotion();
		  }
		  emergency = true;
	  }
	  error = motorcontroller[i]->getSensorError();
	  if (error != EventCode::NONE) {
		  if (motorcontroller[i]->getMotionStateTarget() != MotionState::STOP) {
			  trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted due to sensor error %s (%d) of motorcontroller %d.",
				  parameters->name, toString(error), (int)error, motorcontroller[i]->getMotorcontrollerID());
			  motorcontroller[i]->disableMotion();
		  }
		  emergency = true;
	  }
  }

  if(reset) {
    reset_state_counter=ROBOT_RESET_TICKS;
    for(int i = 0;i<ROBOT_NUM_SLAVE;i++) {
      slave[i]->reset();
    }
	for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
		motorcontroller[i]->reset();
	}
    if(mft!=NULL) {
      mft->reset();
    }
  }
  if(emergency) {
    for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
      if(slave[i]->getMotionStateTarget()!=MotionState::STOP) {
        trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted of slave %d due to emergency.",
          parameters->name, slave[i]->getConnectionID());
        slave[i]->disableMotion();
      }
    }
	for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
		if (motorcontroller[i]->getMotionStateTarget() != MotionState::STOP) {
			trace->Log(tlError, "%s Robot::checkLastErrorAndReset(): Motion aborted of motorcontroller %d due to emergency.",
				parameters->name, motorcontroller[i]->getMotorcontrollerID());
			motorcontroller[i]->disableMotion();
		}
	}
  }
  return reset;
}

bool Robot::hasWarning() {
  bool warning=false;
  for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
    if(slave[i]->getSensorWarning()!= EventCode::NONE ||
      slave[i]->getMotionWarning()!= EventCode::NONE) {
      warning=true;
    }
  }
  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
	  if (motorcontroller[i]->getSensorWarning() != EventCode::NONE ||
		  motorcontroller[i]->getMotionWarning() != EventCode::NONE) {
		  warning = true;
	  }
  }
  return warning;
}

bool Robot::isMotionEnabled() {
  for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
    if(slave[i]->isMotionEnabled())
      return true;
  }
  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
	  if (motorcontroller[i]->isMotionEnabled())
		  return true;
  }
  return false;
}

void Robot::enableMotion() {
  if(isMotionEnabled()) {
    trace->Log(tlInfo, "%s Robot::enableMotion(): Motion already enabled. Ignoring request.", parameters->name);
  }
  else {
	event_handler->acknowledge();
	bool enable = true;
    for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
      if(!slave[i]->canEnableMotion()) {
        trace->Log(tlError, "%s Robot::enableMotion(): Motion not enabled due to slave %d.", parameters->name, slave[i]->getConnectionID());
        enable=false;
        break;
      }
    }
	for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
		if (!motorcontroller[i]->canEnableMotion()) {
			trace->Log(tlError, "%s Robot::enableMotion(): Motion not enabled due to motorcontroller %d.", parameters->name, motorcontroller[i]->getMotorcontrollerID());
			enable = false;
			break;
		}
	}
    if(mft!=NULL && !mft->canEnableMotion()) {
      trace->Log(tlError, "%s Robot::enableMotion(): Motion not enabled due to slave MFT.", parameters->name);
      enable=false;
    }
	if (outside_workspace_error) {
		trace->Log(tlError, "Robot::enableMotion(): Motion not enabled because robot is outside workspace.\n Current position: [%.2f, %.2f, %.2f].\n Limits: x: [%.2f, %.2f], y: [%.2f, %.2f], z: [%.2f, %.2f].",
			outside_workspace_cartesian_position[0], outside_workspace_cartesian_position[1], outside_workspace_cartesian_position[2],
			limits->workspace_limits[0], limits->workspace_limits[1], limits->workspace_limits[2],
			limits->workspace_limits[3], limits->workspace_limits[4], limits->workspace_limits[5]);
		enable = false;
	}
    //Check if enabling is possible
    if(enable) {
      trace->Log(tlInfo, "%s Robot::enableMotion(): Enabling motion", parameters->name);
      enable=true;
      for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
        if(!slave[i]->enableMotion()) {
          trace->Log(tlError, "%s Robot::enableMotion(): Failed to enable motion due to slave %d", parameters->name, slave[i]->getConnectionID());
          enable=false;
          break;
        }
      }
	  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
		  if (!motorcontroller[i]->enableMotion()) {
			  trace->Log(tlError, "%s Robot::enableMotion(): Failed to enable motion due to motorcontroller %d", parameters->name, motorcontroller[i]->getMotorcontrollerID());
			  enable = false;
			  break;
		  }
	  }
      if(enable) {
        if(mft!=NULL) {
		  mft->enableMotion();
        }
      }
      else {
        disableMotion();
      }
    }
  }
}

void Robot::performSoftStop() {
  if(soft_stop_counter==0 && isMotionEnabled()) {
    trace->Log(tlInfo, "%s Robot::performSoftStop()", parameters->name);
    soft_stop_counter=ROBOT_SOFT_STOP_TICKS;
    if(mft!=NULL) {
      mft->setLEDState(LEDState::RED);
    }
  }
  else {
    //If the soft stop conditions are not satisfied, perform a hard stop, just
    //to be sure.
    disableMotion();
  }
}

void Robot::disableMotion() {
	if (disableMotion_output_timer == 0) {
		disableMotion_output_timer = OUTPUT_COOLDOWN_TIME;
		if (!isMotionEnabled()) {
    trace->Log(tlInfo, "%s Robot::disableMotion(): Motion already disabled; disabling anyway.", parameters->name);
  }
		trace->Log(tlInfo, "%s Robot::disableMotion()", parameters->name);
	}
  for(int i = 0;i<ROBOT_NUM_SLAVE;i++) {
    slave[i]->disableMotion();
  }
  for (int i = 0; i<ROBOT_NUM_MOTORCONTROLLER; i++) {
	  motorcontroller[i]->disableMotion();
  }
  if(mft!=NULL) {
	  mft->disableMotion();
  }
}

bool Robot::hasValidInput() {
  for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
    if(!slave[i]->hasValidInput()) {
      return false;
    }
  }
  return true;
}

void Robot::AdsReadCon(AmsAddr& rAddr, ULONG invokeId, ULONG nResult, ULONG cbLength, PVOID pData) {
  int slave_index=-1;
  for(int i=0;i<ROBOT_NUM_SLAVE;i++) {
    if(invokeId==parameters->ksoe_slave_parameters[i].slave_address) {
      slave_index=i;
      break;
    }
  }
  if(slave_index==-1) {
    trace->Log(tlError, "%s Robot::AdsReadCon(): Ads request for unknown slave (invokeId=%d, nResult=0x%08x, cbLength=%d, pData=0x%p)",
      parameters->name, invokeId, nResult, cbLength, pData);
  }
  else {
    if(nResult==ADSERR_NOERR && pData!=NULL) {
      USHORT temperature=*((WORD*)pData);
      //trace->Log(tlVerbose, "%s Robot::AdsReadCon(): Motor temperature %d=%d K",
      //  parameters->name, parameters->ksoe_slave_parameters[slave_index].connection_id, temperature);
      slave[slave_index]->setMotorTemperature(temperature);
    }
    else {
      //Set an invalid temperature to signalize an error.
      slave[slave_index]->setMotorTemperature(0);
      trace->Log(tlError, "%s Robot::AdsReadCon(): Ads request failed (invokeId=%d, nResult=0x%08x, cbLength=%d, pData=0x%p)",
        parameters->name, invokeId, nResult, cbLength, pData);
    }
  }
}

KsoeState Robot::getCollectiveState() {
  KsoeState state=slave[0]->getState();
  for(int i=1;i<ROBOT_NUM_SLAVE;i++) {
    if(state!=slave[i]->getState()) {
      state=KsoeState::NONE;
      break;
    }
  }
  return state;
}


MotorcontrollerModeOfOperation Robot::getCollectiveModeOfOperation() {
	MotorcontrollerModeOfOperation mode_of_operation = motorcontroller[0]->getModeOfOperation();
	for (int i = 1; i < ROBOT_NUM_MOTORCONTROLLER; i++) {
		if (mode_of_operation != motorcontroller[i]->getModeOfOperation()) {
			mode_of_operation = MotorcontrollerModeOfOperation::NONE;
			break;
		}
	}
	return mode_of_operation;
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withLinAxes\gravitational_forces.hpp
void Robot::gravitationVector9DOF(double *q, double *g)
{
	const double G = 9.81;

	const double d1 = 0.1575;
	const double d2 = 0.2025;
	const double d3 = 0.2375;
	const double d4 = 0.1825;
	const double d5 = 0.2175;
	const double d6 = 0.1825;
	const double d7 = 0.081;
	const double d8 = 0.071;

	const double m1 = 6.495;
	const double m2 = 8.807;
	const double m3 = 2.8;
	const double m4 = 5.283;
	const double m5 = 1.889;
	const double m6 = 2.32;
	const double m7 = 0.863;
	const double mt = 0;

	const double sp1x = 0;
	const double sp1y = -0.014390000000000;
	const double sp1z = 0.107503000000000;

	const double sp2x = 1.200000000000000e-05;
	const double sp2y = 0.064720000000000;
	const double sp2z = 0.004804000000000;

	const double sp3x = -2.080000000000000e-04;
	const double sp3y = 0.016010000000000;
	const double sp3z = 0.0822830000000000;

	const double sp4x = -2.030000000000000e-04;
	const double sp4y = 0.098639000000000;
	const double sp4z = 0.011478000000000;

	const double sp5x = 5.700000000000000e-05;
	const double sp5y = 0.014468000000000;
	const double sp5z = 0.0579380000000000;

	const double sp6x = 3.630000000000000e-04;
	const double sp6y = 0.0169820000000000;
	const double sp6z = -0.0197310000000000;

	const double sp7x = 0.001904000000000;
	const double sp7y = -9.300000000000000e-05;
	const double sp7z = 0.0174619999999999;

	const double sptx = 0;
	const double spty = 0;
	const double sptz = 0;

	const double mx = 2.73 + 2 + 9.05 - 2.571 + 169.3 * 0.107;
	const double my = 2.571;
	const double m0 = 9;

	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];

	double t1;
	double t104;
	double t105;
	double t106;
	double t112;
	double t114;
	double t115;
	double t116;
	double t118;
	double t120;
	double t121;
	double t123;
	double t124;
	double t126;
	double t127;
	double t128;
	double t13;
	double t130;
	double t131;
	double t132;
	double t133;
	double t134;
	double t135;
	double t14;
	double t141;
	double t142;
	double t148;
	double t149;
	double t15;
	double t150;
	double t152;
	double t153;
	double t155;
	double t157;
	double t158;
	double t159;
	double t16;
	double t167;
	double t168;
	double t180;
	double t19;
	double t193;
	double t20;
	double t202;
	double t21;
	double t22;
	double t221;
	double t229;
	double t23;
	double t233;
	double t24;
	double t25;
	double t252;
	double t256;
	double t258;
	double t259;
	double t26;
	double t260;
	double t262;
	double t27;
	double t270;
	double t273;
	double t277;
	double t29;
	double t298;
	double t3;
	double t301;
	double t307;
	double t309;
	double t31;
	double t312;
	double t32;
	double t321;
	double t325;
	double t329;
	double t33;
	double t332;
	double t34;
	double t35;
	double t355;
	double t356;
	double t360;
	double t362;
	double t368;
	double t369;
	double t371;
	double t374;
	double t375;
	double t387;
	double t389;
	double t39;
	double t392;
	double t398;
	double t41;
	double t42;
	double t43;
	double t44;
	double t45;
	double t50;
	double t51;
	double t52;
	double t53;
	double t54;
	double t55;
	double t56;
	double t58;
	double t59;
	double t60;
	double t61;
	double t62;
	double t63;
	double t64;
	double t65;
	double t66;
	double t67;
	double t68;
	double t69;
	double t7;
	double t70;
	double t71;
	double t72;
	double t73;
	double t75;
	double t76;
	double t77;
	double t78;
	double t79;
	double t8;
	double t80;
	double t81;
	double t82;
	double t83;
	double t84;
	double t85;
	double t86;
	double t87;
	double t88;
	double t89;
	double t90;
	double t91;
	double t92;
	double t93;
	double t94;
	double t95;
	double t96;
	double t97;
	double t98;
	g[0] = 0;
	g[1] = 0;
	g[2] = 0;
	g[3] = 0;
	g[4] = 0;
	g[5] = 0;
	g[6] = 0;
	g[7] = 0;
	g[8] = 0;
	t1 = m0 + m1 + m2 + m3 + m4 + m5 + m6 + m7 + mt + mx + my;
	t3 = sin_(pitch);
	g[0] = -t3 * t1 * G;
	t7 = sin_(roll);
	t8 = cos_(pitch);
	g[1] = -t8 * t7 * G * (-mt - m7 - m6 - m5 - m4 - m3 - m2 - m1 - m0 - my);
	t13 = -m7 * sp7x - sptx * mt;
	t14 = q[8];
	t15 = cos_(t14);
	t16 = t15 * t13;
	t19 = m7 * sp7y + mt * spty;
	t20 = sin_(t14);
	t21 = t20 * t19;
	t22 = m6 * sp6x;
	t23 = t16 + t21 + t22;
	t24 = q[7];
	t25 = sin_(t24);
	t26 = t25 * t23;
	t27 = d7 + sptz + d8;
	t29 = d7 + sp7z;
	t31 = m6 * sp6y;
	t32 = m7 * t29 + mt * t27 + t31;
	t33 = cos_(t24);
	t34 = t33 * t32;
	t35 = d5 + d6;
	t39 = d5 + sp5z;
	t41 = sp4y * m4;
	t42 = m5 * t39 + m6 * t35 + m7 * t35 + mt * t35 + t26 + t34 + t41;
	t43 = q[5];
	t44 = sin_(t43);
	t45 = t44 * t42;
	t50 = -m7 * t29 - mt * t27 - t31;
	t51 = t25 * t50;
	t52 = t33 * t23;
	t53 = sp5x * m5;
	t54 = t51 + t52 + t53;
	t55 = q[6];
	t56 = cos_(t55);
	t58 = t15 * t19;
	t59 = -t13;
	t60 = t20 * t59;
	t61 = sp5y * m5;
	t62 = m6 * sp6z;
	t63 = t58 + t60 - t61 + t62;
	t64 = sin_(t55);
	t65 = t64 * t63;
	t66 = sp4x * m4;
	t67 = t54 * t56 + t65 - t66;
	t68 = cos_(t43);
	t69 = t68 * t67;
	t70 = sp3x * m3;
	t71 = t45 + t69 - t70;
	t72 = q[4];
	t73 = cos_(t72);
	t75 = t56 * t63;
	t76 = t32 * t64;
	t77 = t25 * t76;
	t78 = t23 * t64;
	t79 = t33 * t78;
	t80 = t64 * t53;
	t81 = sp3y * m3;
	t82 = sp4z * m4;
	t83 = t75 + t77 - t79 - t80 + t81 - t82;
	t84 = sin_(t72);
	t85 = t84 * t83;
	t86 = sp2x * m2;
	t87 = t71 * t73 + t85 + t86;
	t88 = q[3];
	t89 = cos_(t88);
	t90 = t89 * t87;
	t91 = t44 * t67;
	t92 = t15 * t59;
	t93 = -t19;
	t94 = t20 * t93;
	t95 = t92 + t94 - t22;
	t96 = t25 * t95;
	t97 = t33 * t50;
	t98 = -t35;
	t104 = -m5 * t39 + m6 * t98 + m7 * t98 + mt * t98 - t41 + t96 + t97;
	t105 = t68 * t104;
	t106 = -d3 - d4;
	t112 = -d3 - sp3z;
	t114 = sp2y * m2;
	t115 = m3 * t112 + m4 * t106 + m5 * t106 + m6 * t106 + m7 * t106 + mt * t106 + t105 - t114 + t91;
	t116 = sin_(t88);
	t118 = sp1x * m1;
	t120 = q[2];
	t121 = sin_(t120);
	t123 = t73 * t83;
	t124 = t42 * t84;
	t126 = t25 * t32;
	t127 = t33 * t95;
	t128 = t126 + t127 - t53;
	t130 = t15 * t93;
	t131 = t20 * t13;
	t132 = t130 + t131 + t61 - t62;
	t133 = t64 * t132;
	t134 = t128 * t56 + t133 + t66;
	t135 = t134 * t84;
	t141 = m3 * sp3x * t84 + sp1y * m1 + sp2z * m2 - t124 * t44 + t135 * t68 + t123;
	t142 = cos_(t120);
	t148 = t44 * t104;
	t149 = t68 * t134;
	t150 = t148 + t149 + t70;
	t152 = t56 * t132;
	t153 = t152 - t77 + t79 + t80 - t81 + t82;
	t155 = t150 * t73 + t153 * t84 - t86;
	t157 = t44 * t134;
	t158 = t68 * t42;
	t159 = -t106;
	t167 = -m3 * t112 + m4 * t159 + m5 * t159 + m6 * t159 + m7 * t159 + mt * t159 + t114 + t157 + t158;
	t168 = t116 * t167;
	g[2] = -(t8 * t7 * (-t121 * (t115 * t116 - t118 + t90) + t142 * t141) + t3 * (-t121 * t141 + t142 * (t155 * t89 + t118 + t168))) * G;
	t180 = cos_(roll);
	g[3] = -(t8 * (t7 * t142 * (t115 * t89 - t116 * t87) + (-t168 + t90) * t180) + t3 * t121 * (-t116 * t155 + t167 * t89)) * G;
	t193 = -t71 * t84 + t123;
	t202 = m3 * sp3x * t73 + t134 * t68 * t73 - t42 * t44 * t73 - t85;
	g[4] = -(t8 * (t7 * (t142 * t193 * t89 + t121 * t202) + t116 * t193 * t180) + t3 * (t142 * t202 + t121 * t89 * (-t150 * t84 + t153 * t73))) * G;
	t221 = t73 * (t158 - t91);
	t229 = -t124 * t68 - t135 * t44;
	t233 = t149 - t45;
	g[5] = -(t8 * (t7 * (t142 * (t89 * t221 + t116 * (t69 - t148)) + t121 * t229) + (t116 * t221 + t233 * t89) * t180) + t3 * (t142 * t229 + t121 * (t89 * t73 * (t105 - t157) + t116 * t233))) * G;
	t252 = -t54 * t64 + t75;
	t256 = t25 * t32 * t56;
	t258 = t33 * t23 * t56;
	t259 = t56 * t53;
	t260 = -t65 + t256 - t258 - t259;
	t262 = t252 * t68 * t73 + t260 * t84;
	t270 = -t128 * t64 + t152;
	t273 = t270 * t68 * t84 + t260 * t73;
	t277 = t44 * t270;
	g[6] = -(t8 * (t7 * (t142 * (t116 * t252 * t44 + t262 * t89) + t121 * t273) + (t116 * t262 + t277 * t89) * t180) + t3 * (t142 * t273 + t121 * (t89 * (t73 * t68 * t270 + t84 * (-t133 - t256 + t258 + t259)) + t116 * t277))) * G;
	t298 = t52 - t126;
	t301 = t56 * (t97 - t26);
	t307 = t25 * t78 + t33 * t76;
	t309 = t73 * (t298 * t44 + t301 * t68) + t84 * t307;
	t312 = t127 - t51;
	t321 = t34 - t96;
	t325 = t321 * t56 * t68 * t84 - t298 * t44 * t84 + t307 * t73;
	t329 = t56 * t321;
	t332 = t298 * t68 + t329 * t44;
	g[7] = -(t8 * (t7 * (t142 * (t89 * t309 + t116 * (t301 * t44 + t312 * t68)) + t121 * t325) + (t116 * t309 + t332 * t89) * t180) + t3 * (t142 * t325 + t121 * (t89 * (t73 * (t312 * t44 + t329 * t68) - t84 * t307) + t116 * t332))) * G;
	t355 = -t131 + t58;
	t356 = t25 * t355;
	t360 = -t21 + t92;
	t362 = t33 * t355 * t56 + t360 * t64;
	t368 = t33 * t355 * t64;
	t369 = t360 * t56 - t368;
	t371 = t73 * (t356 * t44 + t362 * t68) + t84 * t369;
	t374 = -t60 + t130;
	t375 = t25 * t374;
	t387 = -t94 + t16;
	t389 = t33 * t374 * t56 + t387 * t64;
	t392 = -t25 * t355 * t44 * t84 + t389 * t68 * t84 + t369 * t73;
	t398 = t356 * t68 + t389 * t44;
	g[8] = -(t8 * (t7 * (t142 * (t89 * t371 + t116 * (t362 * t44 + t375 * t68)) + t121 * t392) + (t116 * t371 + t398 * t89) * t180) + t3 * (t142 * t392 + t121 * (t89 * (t73 * (t375 * t44 + t389 * t68) + t84 * (t387 * t56 + t368)) + t116 * t398))) * G;
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withoutLinAxes\gravitational_forces.hpp
void Robot::gravitationVector7DOF(double *q, double *g) {
	const double G = 9.81;

	const double d1 = 0.1575;
	const double d2 = 0.2025;
	const double d3 = 0.2375;
	const double d4 = 0.1825;
	const double d5 = 0.2175;
	const double d6 = 0.1825;
	const double d7 = 0.081;
	const double d8 = 0.071;

	const double m1 = 6.495;
	const double m2 = 8.807;
	const double m3 = 2.8;
	const double m4 = 5.283;
	const double m5 = 1.889;
	const double m6 = 2.32;
	const double m7 = 0.863;
	const double mt = 0;

	const double sp1x = 0;
	const double sp1y = -0.014390000000000;
	const double sp1z = 0.107503000000000;

	const double sp2x = 1.200000000000000e-05;
	const double sp2y = 0.064720000000000;
	const double sp2z = 0.004804000000000;

	const double sp3x = -2.080000000000000e-04;
	const double sp3y = 0.016010000000000;
	const double sp3z = 0.0822830000000000;

	const double sp4x = -2.030000000000000e-04;
	const double sp4y = 0.098639000000000;
	const double sp4z = 0.011478000000000;

	const double sp5x = 5.700000000000000e-05;
	const double sp5y = 0.014468000000000;
	const double sp5z = 0.0579380000000000;

	const double sp6x = 3.630000000000000e-04;
	const double sp6y = 0.0169820000000000;
	const double sp6z = -0.0197310000000000;

	const double sp7x = 0.001904000000000;
	const double sp7y = -9.300000000000000e-05;
	const double sp7z = 0.0174619999999999;

	const double sptx = 0;
	const double spty = 0;
	const double sptz = 0;

	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];

	double t10;
	double t100;
	double t101;
	double t102;
	double t103;
	double t104;
	double t106;
	double t107;
	double t109;
	double t11;
	double t110;
	double t111;
	double t112;
	double t114;
	double t115;
	double t116;
	double t12;
	double t122;
	double t124;
	double t125;
	double t126;
	double t127;
	double t128;
	double t13;
	double t132;
	double t134;
	double t136;
	double t137;
	double t138;
	double t139;
	double t14;
	double t141;
	double t142;
	double t143;
	double t144;
	double t146;
	double t147;
	double t148;
	double t149;
	double t15;
	double t150;
	double t151;
	double t159;
	double t16;
	double t165;
	double t17;
	double t174;
	double t18;
	double t190;
	double t20;
	double t200;
	double t213;
	double t218;
	double t22;
	double t226;
	double t23;
	double t24;
	double t242;
	double t244;
	double t246;
	double t247;
	double t25;
	double t250;
	double t253;
	double t26;
	double t261;
	double t269;
	double t27;
	double t273;
	double t28;
	double t29;
	double t290;
	double t292;
	double t296;
	double t299;
	double t3;
	double t30;
	double t301;
	double t31;
	double t312;
	double t32;
	double t321;
	double t326;
	double t33;
	double t34;
	double t343;
	double t345;
	double t347;
	double t348;
	double t35;
	double t350;
	double t353;
	double t356;
	double t358;
	double t36;
	double t361;
	double t363;
	double t375;
	double t38;
	double t386;
	double t39;
	double t391;
	double t4;
	double t40;
	double t41;
	double t42;
	double t43;
	double t44;
	double t45;
	double t46;
	double t47;
	double t5;
	double t51;
	double t53;
	double t54;
	double t55;
	double t56;
	double t57;
	double t58;
	double t6;
	double t60;
	double t61;
	double t62;
	double t63;
	double t64;
	double t65;
	double t66;
	double t67;
	double t69;
	double t70;
	double t71;
	double t72;
	double t73;
	double t74;
	double t75;
	double t76;
	double t82;
	double t83;
	double t84;
	double t86;
	double t87;
	double t9;
	double t92;
	double t93;
	double t94;

	g[0] = 0;
	g[1] = 0;
	g[2] = 0;
	g[3] = 0;
	g[4] = 0;
	g[5] = 0;
	g[6] = 0;
	t3 = m7 * sp7y + mt * spty;
	t4 = q[6];
	t5 = cos_(t4);
	t6 = t5 * t3;
	t9 = m7 * sp7x + mt * sptx;
	t10 = sin_(t4);
	t11 = t10 * t9;
	t12 = m5 * sp5y;
	t13 = m6 * sp6z;
	t14 = t6 + t11 - t12 + t13;
	t15 = q[4];
	t16 = cos_(t15);
	t17 = t16 * t14;
	t18 = d7 + sptz + d8;
	t20 = d7 + sp7z;
	t22 = m6 * sp6y;
	t23 = m7 * t20 + mt * t18 + t22;
	t24 = sin_(t15);
	t25 = t24 * t23;
	t26 = q[5];
	t27 = sin_(t26);
	t28 = t27 * t25;
	t29 = -t9;
	t30 = t5 * t29;
	t31 = t10 * t3;
	t32 = m6 * sp6x;
	t33 = t30 + t31 + t32;
	t34 = t24 * t33;
	t35 = cos_(t26);
	t36 = t35 * t34;
	t38 = sp5x * m5 * t24;
	t39 = m3 * sp3y;
	t40 = m4 * sp4z;
	t41 = t17 + t28 - t36 - t38 + t39 - t40;
	t42 = q[2];
	t43 = cos_(t42);
	t44 = t43 * t41;
	t45 = t27 * t33;
	t46 = t35 * t23;
	t47 = d5 + d6;
	t51 = d5 + sp5z;
	t53 = m4 * sp4y;
	t54 = m5 * t51 + m6 * t47 + m7 * t47 + mt * t47 + t45 + t46 + t53;
	t55 = sin_(t42);
	t56 = t55 * t54;
	t57 = q[3];
	t58 = sin_(t57);
	t60 = t27 * t23;
	t61 = t5 * t9;
	t62 = -t3;
	t63 = t10 * t62;
	t64 = t61 + t63 - t32;
	t65 = t35 * t64;
	t66 = m5 * sp5x;
	t67 = t60 + t65 - t66;
	t69 = t5 * t62;
	t70 = t10 * t29;
	t71 = t69 + t70 + t12 - t13;
	t72 = t24 * t71;
	t73 = m4 * sp4x;
	t74 = t16 * t67 + t72 + t73;
	t75 = t55 * t74;
	t76 = cos_(t57);
	t82 = m3 * sp3x * t55 + m1 * sp1y + m2 * sp2z - t56 * t58 + t75 * t76 + t44;
	t83 = q[0];
	t84 = sin_(t83);
	t86 = cos_(t83);
	t87 = t27 * t64;
	t92 = -m7 * t20 - mt * t18 - t22;
	t93 = t35 * t92;
	t94 = -t47;
	t100 = -m5 * t51 + m6 * t94 + m7 * t94 + mt * t94 - t53 + t87 + t93;
	t101 = t58 * t100;
	t102 = t76 * t74;
	t103 = m3 * sp3x;
	t104 = t101 + t102 + t103;
	t106 = t16 * t71;
	t107 = t106 - t28 + t36 + t38 - t39 + t40;
	t109 = m2 * sp2x;
	t110 = t104 * t43 + t107 * t55 - t109;
	t111 = q[1];
	t112 = cos_(t111);
	t114 = t58 * t74;
	t115 = t76 * t54;
	t116 = d3 + d4;
	t122 = d3 + sp3z;
	t124 = m2 * sp2y;
	t125 = m3 * t122 + m4 * t116 + m5 * t116 + m6 * t116 + m7 * t116 + mt * t116 + t114 + t115 + t124;
	t126 = sin_(t111);
	t127 = t126 * t125;
	t128 = m1 * sp1x;
	t132 = sin_(roll);
	t134 = cos_(pitch);
	t136 = t58 * t54;
	t137 = t27 * t92;
	t138 = t35 * t33;
	t139 = t137 + t138 + t66;
	t141 = t24 * t14;
	t142 = t139 * t16 + t141 - t73;
	t143 = t76 * t142;
	t144 = t136 + t143 - t103;
	t146 = t55 * t41;
	t147 = t144 * t43 + t109 + t146;
	t148 = t112 * t147;
	t149 = t58 * t142;
	t150 = t76 * t100;
	t151 = -t116;
	t159 = -m3 * t122 + m4 * t151 + m5 * t151 + m6 * t151 + m7 * t151 + mt * t151 - t124 + t149 + t150;
	t165 = sin_(pitch);
	g[0] = (t134 * t132 * (-t84 * t82 + (t110 * t112 + t127 + t128) * t86) + t165 * (-t84 * (t126 * t159 - t128 + t148) + t86 * t82)) * G;
	t174 = cos_(roll);
	g[1] = (t134 * (t132 * (-t110 * t126 + t112 * t125) * t84 + t174 * (-t127 + t148)) + t165 * t86 * (t112 * t159 - t126 * t147)) * G;
	t190 = m3 * sp3x * t43 - t43 * t54 * t58 + t43 * t74 * t76 - t146;
	t200 = -t144 * t55 + t44;
	g[2] = (t134 * (t132 * (t86 * t190 + t112 * (-t104 * t55 + t107 * t43) * t84) + t174 * t126 * t200) + t165 * (t112 * t200 * t86 + t190 * t84)) * G;
	t213 = -t56 * t76 - t58 * t75;
	t218 = t102 - t136;
	t226 = t43 * (t115 - t149);
	g[3] = (t134 * (t132 * (t86 * t213 + (t112 * t43 * (t150 - t114) + t126 * t218) * t84) + t174 * (t112 * t218 + t126 * t226)) + t165 * (t86 * (t112 * t226 + t126 * (t143 - t101)) + t84 * t213)) * G;
	t242 = t27 * t16 * t23;
	t244 = t35 * t16 * t33;
	t246 = sp5x * m5 * t16;
	t247 = -t141 + t242 - t244 - t246;
	t250 = -t24 * t67 + t106;
	t253 = t250 * t55 * t76 + t247 * t43;
	t261 = t58 * t250;
	t269 = -t139 * t24 + t17;
	t273 = t269 * t43 * t76 + t247 * t55;
	g[4] = (t134 * (t132 * (t86 * t253 + (t112 * (t43 * t76 * t250 + t55 * (-t72 - t242 + t244 + t246)) + t126 * t261) * t84) + t174 * (t112 * t261 + t126 * t273)) + t165 * (t86 * (t126 * t269 * t58 + t112 * t273) + t84 * t253)) * G;
	t290 = t25 * t35 + t27 * t34;
	t292 = -t60 + t138;
	t296 = t16 * (t46 - t87);
	t299 = -t292 * t55 * t58 + t296 * t55 * t76 + t290 * t43;
	t301 = t65 - t137;
	t312 = t292 * t76 + t296 * t58;
	t321 = t16 * (t93 - t45);
	t326 = t43 * (t292 * t58 + t321 * t76) + t55 * t290;
	g[5] = (t134 * (t132 * (t86 * t299 + (t112 * (t43 * (t296 * t76 + t301 * t58) - t55 * t290) + t126 * t312) * t84) + t174 * (t112 * t312 + t126 * t326)) + t165 * (t86 * (t112 * t326 + t126 * (t301 * t76 + t321 * t58)) + t84 * t299)) * G;
	t343 = -t31 + t61;
	t345 = -t70 + t6;
	t347 = t35 * t24 * t345;
	t348 = t16 * t343 - t347;
	t350 = t27 * t345;
	t353 = -t11 + t69;
	t356 = -t63 + t30;
	t358 = t16 * t35 * t353 + t24 * t356;
	t361 = -t350 * t55 * t58 + t358 * t55 * t76 + t348 * t43;
	t363 = t27 * t353;
	t375 = t350 * t76 + t358 * t58;
	t386 = t16 * t345 * t35 + t24 * t343;
	t391 = t43 * (t350 * t58 + t386 * t76) + t55 * t348;
	g[6] = (t134 * (t132 * (t86 * t361 + (t112 * (t43 * (t358 * t76 + t363 * t58) + t55 * (t16 * t356 + t347)) + t126 * t375) * t84) + t174 * (t112 * t375 + t126 * t391)) + t165 * (t86 * (t112 * t391 + t126 * (t363 * t76 + t386 * t58)) + t84 * t361)) * G;
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withLinAxes\geo_jacobian_position.hpp
void Robot::endeffectorVelocity9DOF(double *q, double *qp, double *cartesian_velocity) {
	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];
	
	double dtex = limits->tool_tcp[0];
	double dtey = limits->tool_tcp[1];
	double dtez = limits->tool_tcp[2];
	double rteyaw = limits->tool_tcp[3];
	double rtepitch = limits->tool_tcp[4];
	double rteroll = limits->tool_tcp[5];

	double d1 = 0.1575;
	double d2 = 0.2025;
	double d3 = 0.2375;
	double d4 = 0.1825;
	double d5 = 0.2175;
	double d6 = 0.1825;
	double d7 = 0.081;
	double d8 = 0.071;

	double Jv[3][9];
	double t1;
	double t10;
	double t103;
	double t105;
	double t107;
	double t108;
	double t11;
	double t110;
	double t112;
	double t114;
	double t117;
	double t12;
	double t121;
	double t126;
	double t13;
	double t134;
	double t14;
	double t145;
	double t15;
	double t152;
	double t157;
	double t159;
	double t16;
	double t160;
	double t165;
	double t167;
	double t175;
	double t18;
	double t181;
	double t187;
	double t19;
	double t192;
	double t195;
	double t197;
	double t2;
	double t20;
	double t200;
	double t205;
	double t206;
	double t208;
	double t212;
	double t217;
	double t22;
	double t227;
	double t232;
	double t234;
	double t238;
	double t24;
	double t242;
	double t244;
	double t247;
	double t25;
	double t258;
	double t26;
	double t263;
	double t267;
	double t27;
	double t272;
	double t273;
	double t280;
	double t281;
	double t282;
	double t284;
	double t285;
	double t292;
	double t297;
	double t3;
	double t30;
	double t302;
	double t306;
	double t307;
	double t31;
	double t313;
	double t317;
	double t32;
	double t321;
	double t327;
	double t33;
	double t331;
	double t333;
	double t335;
	double t336;
	double t34;
	double t340;
	double t341;
	double t344;
	double t350;
	double t358;
	double t362;
	double t364;
	double t365;
	double t368;
	double t369;
	double t370;
	double t374;
	double t375;
	double t376;
	double t377;
	double t382;
	double t388;
	double t39;
	double t390;
	double t394;
	double t395;
	double t4;
	double t40;
	double t406;
	double t408;
	double t41;
	double t412;
	double t416;
	double t418;
	double t419;
	double t42;
	double t422;
	double t423;
	double t428;
	double t43;
	double t431;
	double t44;
	double t446;
	double t450;
	double t453;
	double t454;
	double t458;
	double t46;
	double t460;
	double t462;
	double t463;
	double t468;
	double t47;
	double t470;
	double t474;
	double t476;
	double t477;
	double t482;
	double t488;
	double t49;
	double t491;
	double t495;
	double t497;
	double t5;
	double t50;
	double t502;
	double t504;
	double t508;
	double t51;
	double t510;
	double t514;
	double t518;
	double t52;
	double t522;
	double t53;
	double t530;
	double t532;
	double t534;
	double t537;
	double t539;
	double t54;
	double t542;
	double t543;
	double t544;
	double t547;
	double t549;
	double t55;
	double t550;
	double t554;
	double t555;
	double t556;
	double t559;
	double t56;
	double t560;
	double t562;
	double t57;
	double t572;
	double t575;
	double t577;
	double t580;
	double t581;
	double t585;
	double t586;
	double t587;
	double t588;
	double t592;
	double t594;
	double t596;
	double t598;
	double t6;
	double t600;
	double t610;
	double t613;
	double t624;
	double t626;
	double t63;
	double t65;
	double t661;
	double t663;
	double t701;
	double t704;
	double t72;
	double t74;
	double t77;
	double t78;
	double t796;
	double t797;
	double t8;
	double t803;
	double t807;
	double t813;
	double t815;
	double t817;
	double t820;
	double t822;
	double t825;
	double t86;
	double t97;
	double t99;
	Jv[0][0] = 0;
	Jv[0][1] = 0;
	Jv[0][2] = 0;
	Jv[0][3] = 0;
	Jv[0][4] = 0;
	Jv[0][5] = 0;
	Jv[0][6] = 0;
	Jv[0][7] = 0;
	Jv[0][8] = 0;
	Jv[1][0] = 0;
	Jv[1][1] = 0;
	Jv[1][2] = 0;
	Jv[1][3] = 0;
	Jv[1][4] = 0;
	Jv[1][5] = 0;
	Jv[1][6] = 0;
	Jv[1][7] = 0;
	Jv[1][8] = 0;
	Jv[2][0] = 0;
	Jv[2][1] = 0;
	Jv[2][2] = 0;
	Jv[2][3] = 0;
	Jv[2][4] = 0;
	Jv[2][5] = 0;
	Jv[2][6] = 0;
	Jv[2][7] = 0;
	Jv[2][8] = 0;
	t1 = cos_(yaw);
	t2 = cos_(pitch);
	Jv[0][0] = t2 * t1;
	t3 = sin_(yaw);
	t4 = cos_(roll);
	t5 = t4 * t3;
	t6 = sin_(pitch);
	t8 = sin_(roll);
	Jv[0][1] = t1 * t6 * t8 - t5;
	t10 = t8 * t6;
	t11 = q[6];
	t12 = cos_(t11);
	t13 = dtez + d7 + d8;
	t14 = t13 * t12;
	t15 = q[7];
	t16 = sin_(t15);
	t18 = cos_(t15);
	t19 = q[8];
	t20 = cos_(t19);
	t22 = sin_(t19);
	t24 = dtex * t20 - dtey * t22;
	t25 = t24 * t18;
	t26 = t12 * t25;
	t27 = sin_(t11);
	t30 = dtex * t22 + dtey * t20;
	t31 = t30 * t27;
	t32 = t14 * t16 + t26 - t31;
	t33 = q[5];
	t34 = cos_(t33);
	t39 = -t13 * t18 + t16 * t24 - d5 - d6;
	t40 = sin_(t33);
	t41 = t40 * t39;
	t42 = t32 * t34 + t41;
	t43 = q[4];
	t44 = cos_(t43);
	t46 = sin_(t43);
	t47 = t13 * t27;
	t49 = t12 * t30;
	t50 = t27 * t18;
	t51 = t24 * t50;
	t52 = t16 * t47 + t49 + t51;
	t53 = t52 * t46;
	t54 = t42 * t44 - t53;
	t55 = q[3];
	t56 = cos_(t55);
	t57 = t56 * t54;
	t63 = sin_(t55);
	t65 = t39 * t8;
	t72 = t40 * t32;
	t74 = t63 * (t72 + d3 + d4);
	t77 = q[2];
	t78 = sin_(t77);
	t86 = t46 * t8;
	t97 = cos_(t77);
	t99 = t78 * (-t57 * t10 - t44 * t52 * t2 + t34 * (-t2 * t32 * t46 + t6 * t63 * t65) - t46 * t39 * t2 * t40 - t10 * t74) + t97 * (t56 * t54 * t2 - t44 * t8 * t52 * t6 + t34 * (-t2 * t39 * t63 - t32 * t6 * t86) - t46 * t65 * t40 * t6 + t2 * t74);
	t103 = t42 * t46 + t44 * t52;
	t105 = -t24;
	t107 = t13 * t18;
	t108 = t105 * t16 + d5 + d6 + t107;
	t110 = t108 * t34 + d3 + d4 + t72;
	t112 = t110 * t63 + t57;
	t114 = t103 * t97 + t112 * t78;
	Jv[0][2] = t1 * t99 + t114 * t5;
	t117 = t97 * t8;
	t121 = t78 * t2;
	t126 = t8 * t110;
	t134 = t63 * (t6 * (t110 * t4 - t117 * t54) - t54 * t121) + (t6 * (t126 * t97 + t4 * t54) + t78 * t110 * t2) * t56;
	t145 = t63 * (-t4 * t54 * t97 - t126) + (t110 * t4 * t97 - t54 * t8) * t56;
	Jv[0][3] = t1 * t134 - t145 * t3;
	t152 = t6 * (t117 * t56 + t4 * t63) + t56 * t121;
	t157 = t6 * t78 * t8 - t2 * t97;
	t159 = t152 * t46 + t157 * t44;
	t160 = t159 * t34;
	t165 = t152 * t44 - t157 * t46;
	t167 = t12 * t160 + t165 * t27;
	t175 = t12 * t165 - t160 * t27;
	t181 = dtey * t40;
	t187 = t13 * t40;
	t192 = d5 + d6;
	t195 = t20 * (-dtex * t159 * t16 * t40 - dtex * t167 * t18 - dtey * t175) + t22 * (dtey * t167 * t18 + t159 * t16 * t181 - dtex * t175) + t18 * t159 * t187 - t16 * t167 * t13 + t159 * t40 * t192;
	t197 = t97 * t46;
	t200 = t197 * t56 + t44 * t78;
	t205 = t78 * t46;
	t206 = t44 * t56 * t97 - t205;
	t208 = t12 * t200 * t34 + t206 * t27;
	t212 = t16 * t200;
	t217 = -t200 * t27 * t34 + t12 * t206;
	t227 = t200 * t40;
	t232 = t20 * (-dtex * t18 * t208 - dtex * t212 * t40 - dtey * t217) + t22 * (dtey * t18 * t208 - dtex * t217 + t181 * t212) + t107 * t227 - t16 * t208 * t13 + t192 * t227;
	t234 = t8 * t63;
	t238 = t12 * t34 * t46 + t27 * t44;
	t242 = t16 * t40;
	t244 = t46 * t34;
	t247 = t12 * t44 - t244 * t27;
	t258 = t46 * t40;
	t263 = t20 * (-dtex * t18 * t238 - dtex * t242 * t46 - dtey * t247) + t22 * (dtey * t18 * t238 + dtey * t242 * t46 - dtex * t247) + t107 * t258 - t16 * t13 * t238 + t192 * t258;
	Jv[0][4] = t234 * t263 * t3 + t1 * t195 - t232 * t5;
	t267 = t16 * t13;
	t272 = t108 * t63;
	t273 = t44 * t272;
	t280 = t12 * (t25 + t267) - t31;
	t281 = t280 * t63;
	t282 = t108 * t44 * t56 - t281;
	t284 = t108 * t205;
	t285 = t282 * t97 - t284;
	t292 = t108 * t46 * t97 + t282 * t78;
	t297 = t44 * t281;
	t302 = t280 * t44 * t56 + t272;
	t306 = -t280 * t46 * t78 + t302 * t97;
	t307 = t8 * t306;
	t313 = t280 * t46 * t97 + t302 * t78;
	t317 = t34 * (t6 * (t4 * (t56 * (t12 * (t105 * t18 - t267) + t31) - t273) - t285 * t8) - t2 * t292) - (t6 * (t4 * (t39 * t56 + t297) + t307) + t2 * t313) * t40;
	t321 = t280 * t56 + t273;
	t327 = t108 * t56 - t297;
	t331 = t34 * (t285 * t4 - t321 * t8) + (t306 * t4 + t327 * t8) * t40;
	Jv[0][5] = t1 * t317 + t3 * t331;
	t333 = t34 * t44;
	t335 = t40 * t63;
	t336 = t333 * t56 + t335;
	t340 = -t34 * t46 * t78 + t336 * t97;
	t341 = t8 * t340;
	t344 = t333 * t63 - t40 * t56;
	t350 = t244 * t97 + t336 * t78;
	t358 = t18 * t34;
	t362 = t24 * t358 * t44 - t30 * t46;
	t364 = t18 * t24 * t40 * t56 - t362 * t63;
	t365 = t364 * t4;
	t368 = t24 * t40;
	t369 = t368 * t63 * t18;
	t370 = t362 * t56 + t369;
	t374 = t24 * t46 * t358;
	t375 = t30 * t44 + t374;
	t376 = t78 * t375;
	t377 = t370 * t97 - t376;
	t382 = t370 * t78;
	t388 = t46 * t63;
	t390 = t200 * t8 + t388 * t4;
	t394 = t44 * t97;
	t395 = -t46 * t56 * t78 + t394;
	t406 = t18 * t24 * t46 + t30 * t34 * t44;
	t408 = -t30 * t40 * t56 + t406 * t63;
	t412 = t30 * t335 + t406 * t56;
	t416 = -t244 * t30 + t25 * t44;
	t418 = t412 * t97 + t416 * t78;
	t419 = t8 * t418;
	t422 = t97 * t416;
	t423 = t412 * t78;
	t428 = t27 * (-t16 * (t6 * (t344 * t4 + t341) + t350 * t2) * t13 + t6 * (-t377 * t8 + t365) - t2 * (t375 * t97 + t382)) - t12 * (t16 * t13 * (-t2 * t395 + t390 * t6) + t6 * (t4 * t408 + t419) - (t422 - t423) * t2);
	t431 = -t344;
	t446 = -t408;
	t450 = t27 * (t16 * (t340 * t4 + t431 * t8) * t13 + t4 * t377 + t8 * t364) + t12 * (t16 * (t200 * t4 - t388 * t8) * t13 + t4 * t418 + t8 * t446);
	Jv[0][6] = t1 * t428 + t3 * t450;
	t453 = t40 * t12;
	t454 = t13 * t453;
	t458 = t14 * t34 + t368;
	t460 = t27 * t46;
	t462 = -t13 * t460 + t44 * t458;
	t463 = t462 * t63;
	t468 = t105 * t34 + t454;
	t470 = t462 * t56 + t468 * t63;
	t474 = t44 * t47 + t458 * t46;
	t476 = t470 * t97 - t474 * t78;
	t477 = t8 * t476;
	t482 = t470 * t78 + t474 * t97;
	t488 = -t12 * t24 * t34 + t187;
	t491 = t24 * t460 + t44 * t488;
	t495 = t13 * t34 + t24 * t453;
	t497 = t491 * t56 - t495 * t63;
	t502 = -t24 * t27 * t44 + t46 * t488;
	t504 = t497 * t97 - t502 * t78;
	t508 = t491 * t63 + t495 * t56;
	t510 = t4 * t508 + t504 * t8;
	t514 = t497 * t78 + t502 * t97;
	t518 = t18 * (t6 * (t4 * (t56 * (t24 * t34 - t454) + t463) + t477) + t482 * t2) + t16 * (t2 * t514 + t510 * t6);
	t522 = t468 * t56 - t463;
	t530 = t18 * (t4 * t476 + t522 * t8) + (t4 * t504 - t508 * t8) * t16;
	Jv[0][7] = t1 * t518 - t3 * t530;
	t532 = t16 * t34;
	t534 = t18 * t12;
	t537 = dtex * t27 + dtey * t534;
	t539 = -dtey * t532 + t40 * t537;
	t542 = dtey * t242;
	t543 = t34 * t537 + t542;
	t544 = t44 * t543;
	t547 = -dtex * t12 + dtey * t50;
	t549 = -t46 * t547 + t544;
	t550 = t549 * t63;
	t554 = -t539;
	t555 = t554 * t63;
	t556 = t549 * t56 - t555;
	t559 = t543 * t46;
	t560 = t44 * t547 + t559;
	t562 = t556 * t97 - t560 * t78;
	t572 = dtex * t532;
	t575 = -dtex * t534 + dtey * t27;
	t577 = -t40 * t575 - t572;
	t580 = dtex * t242;
	t581 = t34 * t575 - t580;
	t585 = dtex * t50 + dtey * t12;
	t586 = t585 * t46;
	t587 = t44 * t581 + t586;
	t588 = t587 * t63;
	t592 = -t577;
	t594 = t56 * t587 + t592 * t63;
	t596 = t44 * t585;
	t598 = -t46 * t581 + t596;
	t600 = t594 * t97 + t598 * t78;
	t610 = t20 * (t6 * (t4 * (t539 * t56 - t550) - t562 * t8) - t2 * (t556 * t78 + t560 * t97)) + (t6 * (t4 * (t56 * t577 + t588) + t600 * t8) - t2 * (-t594 * t78 + t598 * t97)) * t22;
	t613 = t56 * t554;
	t624 = t20 * (t4 * t562 - (t613 + t550) * t8) - (t4 * t600 + (t56 * t592 - t588) * t8) * t22;
	Jv[0][8] = t1 * t610 + t3 * t624;
	Jv[1][0] = t2 * t3;
	t626 = t4 * t1;
	Jv[1][1] = t3 * t6 * t8 + t626;
	Jv[1][2] = -t114 * t626 + t3 * t99;
	Jv[1][3] = t1 * t145 + t134 * t3;
	Jv[1][4] = -t1 * t234 * t263 + t195 * t3 + t232 * t626;
	Jv[1][5] = -t1 * t331 + t3 * t317;
	Jv[1][6] = -t1 * t450 + t3 * t428;
	Jv[1][7] = t530 * t1 + t3 * t518;
	Jv[1][8] = -t1 * t624 + t3 * t610;
	Jv[2][0] = -t6;
	Jv[2][1] = t8 * t2;
	Jv[2][2] = -t2 * t8 * t114 - t6 * (-t103 * t78 + t112 * t97);
	t661 = d3 + d4;
	t663 = t49 + t51;
	Jv[2][3] = t2 * (t63 * (t13 * t16 * t197 * t27 * t8 + t32 * t4 * t40 - t34 * t39 * t4 - t394 * t42 * t8 + t663 * t86 * t97 + t4 * t661) + (-t13 * t16 * t4 * t460 + t32 * t40 * t8 * t97 - t34 * t65 * t97 + t4 * t42 * t44 - t4 * t46 * t663 + t117 * t661) * t56) - t6 * t78 * (t63 * (t44 * (-t32 * t34 - t41) + t53) + t56 * t110);
	t701 = -t16 * t458 + t34 * (-t26 + t31) + (t107 + d5 + d6) * t40;
	t704 = t52 * t78;
	Jv[2][4] = t2 * (t46 * (t8 * (t56 * t701 * t97 + t704) + t4 * t701 * t63) + (t8 * (-t52 * t56 * t97 + t701 * t78) - t4 * t52 * t63) * t44) + (t46 * (-t56 * t701 * t78 + t52 * t97) + t44 * (t56 * t704 + t701 * t97)) * t6;
	Jv[2][5] = t2 * (t34 * (t8 * (-t282 * t97 + t284) - t4 * t321) - (-t327 * t4 + t307) * t40) + (t292 * t34 + t313 * t40) * t6;
	Jv[2][6] = t2 * (t27 * (-t16 * (-t4 * t431 + t341) * t13 + t8 * (t97 * (-t362 * t56 - t369) + t376) + t365) - t12 * (t13 * t16 * t390 - t4 * t446 + t419)) - (t27 * (-t16 * t350 * t13 + t97 * (-t30 * t44 - t374) - t382) + t12 * (t13 * t16 * t395 + t422 - t423)) * t6;
	Jv[2][7] = t2 * (t18 * (-t4 * t522 + t477) + t16 * t510) - (t16 * t514 + t18 * t482) * t6;
	t796 = -t547;
	t797 = t796 * t46;
	t803 = t44 * t796 - t559;
	t807 = t544 + t797;
	t813 = -t575;
	t815 = t34 * t813 + t580;
	t817 = t44 * t815 - t586;
	t820 = -t40 * t813 + t572;
	t822 = t56 * t817 - t63 * t820;
	t825 = t46 * t815 + t596;
	Jv[2][8] = t2 * (t20 * (t8 * (t97 * (t56 * (t44 * (-t34 * t537 - t542) - t797) + t555) - t78 * t803) - (t63 * t807 + t613) * t4) - (t8 * (-t78 * t825 + t822 * t97) + t4 * (t56 * t820 + t63 * t817)) * t22) - (t20 * (t97 * t803 - t78 * (t56 * t807 - t555)) - (t78 * t822 + t825 * t97) * t22) * t6;

	cartesian_velocity[0] = 0;
	cartesian_velocity[1] = 0;
	cartesian_velocity[2] = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < ROBOT_NUM_AXIS; j++) {
			cartesian_velocity[i] += Jv[i][j] * qp[j];
		}
	}
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withoutLinAxes\geo_jacobian_position.hpp
void Robot::endeffectorVelocity7DOF(double *q, double *qp, double *cartesian_velocity) {
	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];

	double dtex = limits->tool_tcp[0];
	double dtey = limits->tool_tcp[1];
	double dtez = limits->tool_tcp[2];
	double rteyaw = limits->tool_tcp[3];
	double rtepitch = limits->tool_tcp[4];
	double rteroll = limits->tool_tcp[5];

	double d1 = 0.1575;
	double d2 = 0.2025;
	double d3 = 0.2375;
	double d4 = 0.1825;
	double d5 = 0.2175;
	double d6 = 0.1825;
	double d7 = 0.081;
	double d8 = 0.071;

	double Jv[3][7];
	double t1;
	double t100;
	double t102;
	double t104;
	double t108;
	double t11;
	double t110;
	double t111;
	double t112;
	double t113;
	double t116;
	double t12;
	double t120;
	double t124;
	double t13;
	double t131;
	double t140;
	double t15;
	double t152;
	double t159;
	double t164;
	double t165;
	double t166;
	double t167;
	double t17;
	double t171;
	double t172;
	double t174;
	double t177;
	double t18;
	double t182;
	double t188;
	double t19;
	double t194;
	double t199;
	double t2;
	double t20;
	double t201;
	double t204;
	double t207;
	double t210;
	double t212;
	double t213;
	double t215;
	double t219;
	double t224;
	double t23;
	double t234;
	double t239;
	double t24;
	double t245;
	double t249;
	double t25;
	double t251;
	double t254;
	double t26;
	double t265;
	double t27;
	double t271;
	double t276;
	double t278;
	double t280;
	double t281;
	double t282;
	double t283;
	double t285;
	double t288;
	double t289;
	double t293;
	double t295;
	double t296;
	double t3;
	double t30;
	double t303;
	double t305;
	double t306;
	double t307;
	double t309;
	double t312;
	double t313;
	double t319;
	double t32;
	double t325;
	double t33;
	double t336;
	double t338;
	double t34;
	double t341;
	double t342;
	double t343;
	double t345;
	double t346;
	double t347;
	double t348;
	double t35;
	double t354;
	double t355;
	double t356;
	double t36;
	double t365;
	double t366;
	double t369;
	double t37;
	double t371;
	double t377;
	double t380;
	double t388;
	double t39;
	double t395;
	double t398;
	double t4;
	double t40;
	double t403;
	double t41;
	double t414;
	double t416;
	double t417;
	double t42;
	double t423;
	double t43;
	double t436;
	double t44;
	double t445;
	double t45;
	double t458;
	double t46;
	double t461;
	double t463;
	double t466;
	double t467;
	double t469;
	double t47;
	double t471;
	double t473;
	double t474;
	double t477;
	double t478;
	double t48;
	double t488;
	double t49;
	double t494;
	double t5;
	double t50;
	double t509;
	double t512;
	double t514;
	double t517;
	double t518;
	double t52;
	double t532;
	double t551;
	double t559;
	double t561;
	double t565;
	double t568;
	double t57;
	double t570;
	double t572;
	double t574;
	double t577;
	double t580;
	double t581;
	double t584;
	double t588;
	double t589;
	double t59;
	double t591;
	double t598;
	double t6;
	double t607;
	double t614;
	double t616;
	double t619;
	double t620;
	double t622;
	double t624;
	double t627;
	double t628;
	double t642;
	double t65;
	double t653;
	double t66;
	double t693;
	double t696;
	double t7;
	double t71;
	double t72;
	double t731;
	double t74;
	double t762;
	double t769;
	double t786;
	double t795;
	double t8;
	double t801;
	double t813;
	double t817;
	double t845;
	double t849;
	double t857;
	double t861;
	double t87;
	double t882;
	double t884;
	double t886;
	double t889;
	double t891;
	double t895;
	double t897;
	double t9;
	double t900;
	double t902;
	double t914;
	double t917;
	double t920;
	double t922;
	double t926;
	double t930;
	double t932;
	double t94;
	double t95;
	double t97;
	double t99;
	Jv[0][0] = 0;
	Jv[0][1] = 0;
	Jv[0][2] = 0;
	Jv[0][3] = 0;
	Jv[0][4] = 0;
	Jv[0][5] = 0;
	Jv[0][6] = 0;
	Jv[1][0] = 0;
	Jv[1][1] = 0;
	Jv[1][2] = 0;
	Jv[1][3] = 0;
	Jv[1][4] = 0;
	Jv[1][5] = 0;
	Jv[1][6] = 0;
	Jv[2][0] = 0;
	Jv[2][1] = 0;
	Jv[2][2] = 0;
	Jv[2][3] = 0;
	Jv[2][4] = 0;
	Jv[2][5] = 0;
	Jv[2][6] = 0;
	t1 = sin_(roll);
	t2 = sin_(pitch);
	t3 = t1 * t2;
	t4 = q[4];
	t5 = cos_(t4);
	t6 = dtez + d7 + d8;
	t7 = t6 * t5;
	t8 = q[5];
	t9 = sin_(t8);
	t11 = cos_(t8);
	t12 = q[6];
	t13 = cos_(t12);
	t15 = sin_(t12);
	t17 = dtex * t13 - dtey * t15;
	t18 = t17 * t11;
	t19 = t5 * t18;
	t20 = sin_(t4);
	t23 = dtex * t15 + dtey * t13;
	t24 = t23 * t20;
	t25 = t7 * t9 + t19 - t24;
	t26 = q[3];
	t27 = cos_(t26);
	t30 = -t6;
	t32 = t11 * t30 + t17 * t9 - d5 - d6;
	t33 = sin_(t26);
	t34 = t33 * t32;
	t35 = t25 * t27 + t34;
	t36 = q[2];
	t37 = cos_(t36);
	t39 = t6 * t20;
	t40 = t9 * t39;
	t41 = t5 * t23;
	t42 = t20 * t11;
	t43 = t17 * t42;
	t44 = t40 + t41 + t43;
	t45 = sin_(t36);
	t46 = t45 * t44;
	t47 = t35 * t37 - t46;
	t48 = q[1];
	t49 = cos_(t48);
	t50 = t49 * t47;
	t52 = cos_(pitch);
	t57 = sin_(t48);
	t59 = t32 * t2;
	t65 = t33 * t25;
	t66 = t65 + d3 + d4;
	t71 = q[0];
	t72 = cos_(t71);
	t74 = sin_(t71);
	t87 = t33 * t1;
	t94 = t72 * (t50 * t3 - t37 * t52 * t44 + t27 * (-t1 * t57 * t59 - t25 * t45 * t52) - t45 * t52 * t34 + t2 * t57 * t1 * t66) - (t49 * t47 * t52 + t37 * t2 * t44 * t1 + t27 * (t1 * t2 * t25 * t45 - t32 * t52 * t57) + t45 * t59 * t87 + t52 * t57 * t66) * t74;
	t95 = cos_(yaw);
	t97 = -t17;
	t99 = t11 * t6;
	t100 = t9 * t97 + d5 + d6 + t99;
	t102 = t100 * t27 + d3 + d4 + t65;
	t104 = t102 * t57 + t50;
	t108 = t35 * t45 + t37 * t44;
	t110 = t104 * t72 - t108 * t74;
	t111 = cos_(roll);
	t112 = t111 * t110;
	t113 = sin_(yaw);
	Jv[0][0] = -t112 * t113 + t94 * t95;
	t116 = -t25;
	t120 = t74 * t1;
	t124 = t72 * t52;
	t131 = t37 * (t116 * t27 - t34) + t46;
	t140 = t57 * (t2 * (t111 * (t116 * t33 + t27 * t32 - d3 - d4) - t47 * t120) - t47 * t124) + t49 * (t2 * (t102 * t120 + t111 * t131) + t72 * t52 * t102);
	t152 = t57 * (-t111 * t47 * t74 + t102 * t1) + (t102 * t111 * t74 + t1 * t47) * t49;
	Jv[0][1] = -t113 * t152 + t140 * t95;
	t159 = t2 * (-t111 * t57 + t120 * t49) + t49 * t124;
	t164 = -t1 * t2 * t72 + t52 * t74;
	t165 = t164 * t37;
	t166 = t159 * t45 + t165;
	t167 = t166 * t27;
	t171 = -t164 * t45;
	t172 = t159 * t37 + t171;
	t174 = t167 * t5 + t172 * t20;
	t177 = t166 * t33;
	t182 = -t167 * t20 + t172 * t5;
	t188 = dtey * t33;
	t194 = t6 * t33;
	t199 = d5 + d6;
	t201 = t13 * (-dtex * t11 * t174 - dtex * t177 * t9 - dtey * t182) + t15 * (dtey * t11 * t174 + t166 * t188 * t9 - dtex * t182) + t11 * t166 * t194 - t9 * t6 * t174 + t199 * t177;
	t204 = t45 * t74;
	t207 = -t204 * t49 + t37 * t72;
	t210 = t49 * t74;
	t212 = t45 * t72;
	t213 = t210 * t37 + t212;
	t215 = t207 * t27 * t5 - t20 * t213;
	t219 = t9 * t207;
	t224 = t20 * t207 * t27 + t213 * t5;
	t234 = t207 * t33;
	t239 = t13 * (-dtex * t11 * t215 - dtex * t219 * t33 + dtey * t224) + t15 * (dtey * t11 * t215 + dtex * t224 + t188 * t219) + t99 * t234 - t9 * t6 * t215 + t199 * t234;
	t245 = t27 * t45 * t5 + t20 * t37;
	t249 = t9 * t33;
	t251 = t45 * t27;
	t254 = -t20 * t251 + t37 * t5;
	t265 = t45 * t33;
	t271 = (t13 * (-dtex * t11 * t245 - dtex * t249 * t45 - dtey * t254) + t15 * (dtey * t11 * t245 + dtey * t249 * t45 - dtex * t254) + t99 * t265 - t9 * t6 * t245 + t199 * t265) * t57;
	Jv[0][2] = -t1 * t113 * t271 + t111 * t113 * t239 + t201 * t95;
	t276 = t5 * (t6 * t9 + t18) - t24;
	t278 = t37 * t100;
	t280 = t276 * t49 + t278 * t57;
	t281 = t111 * t280;
	t282 = t74 * t100;
	t283 = t49 * t37;
	t285 = t276 * t57;
	t288 = t72 * t45 * t100;
	t289 = t282 * t283 - t285 * t74 + t288;
	t293 = t100 * t72;
	t295 = t45 * t282;
	t296 = t57 * t72;
	t303 = t37 * t276;
	t305 = t100 * t49 - t303 * t57;
	t306 = t111 * t305;
	t307 = t276 * t74;
	t309 = t100 * t57;
	t312 = t72 * t45 * t276;
	t313 = t283 * t307 + t309 * t74 + t312;
	t319 = t45 * t307;
	t325 = t27 * (t2 * (-t1 * t289 + t281) - (-t276 * t296 + t283 * t293 - t295) * t52) - (t2 * (t1 * t313 + t306) + (t276 * t283 * t72 + t293 * t57 - t319) * t52) * t33;
	t336 = t27 * (t1 * t280 + t111 * t289) - t33 * (t1 * t305 - t111 * t313);
	Jv[0][3] = t113 * t336 + t325 * t95;
	t338 = t27 * t37;
	t341 = t33 * t49 - t338 * t57;
	t342 = t111 * t341;
	t343 = t74 * t27;
	t345 = t72 * t251;
	t346 = t57 * t74;
	t347 = t33 * t346;
	t348 = t283 * t343 + t345 + t347;
	t354 = t45 * t343;
	t355 = t33 * t57;
	t356 = t72 * t355;
	t365 = t11 * t27;
	t366 = t37 * t17;
	t369 = -t23 * t45 + t365 * t366;
	t371 = -t11 * t17 * t33 * t49 + t369 * t57;
	t377 = t345 + t347;
	t380 = t11 * t17 * t377 + t23 * t37 * t72 + t369 * t49 * t74;
	t388 = t356 - t354;
	t395 = t45 * t57;
	t398 = -t1 * t207 - t111 * t395;
	t403 = t45 * t49 * t72 + t37 * t74;
	t414 = t11 * t17 * t45 + t23 * t27 * t37;
	t416 = t23 * t33 * t49 - t414 * t57;
	t417 = t111 * t416;
	t423 = t11 * t366 * t72 - t414 * t49 * t74 - t23 * t377;
	t436 = t20 * (-t9 * t6 * (t2 * (t1 * t348 + t342) + (t27 * t283 * t72 - t354 + t356) * t52) + t2 * (-t1 * t380 + t111 * t371) - t52 * (t11 * t17 * t388 - t23 * t37 * t74 + t369 * t49 * t72)) - (t9 * t6 * (t2 * t398 + t403 * t52) + t2 * (-t1 * t423 + t417) + t52 * (t11 * t366 * t74 + t414 * t49 * t72 + t23 * t388)) * t5;
	t445 = -t371;
	t458 = t20 * (-t9 * t6 * (-t1 * t341 + t111 * t348) - t111 * t380 + t1 * t445) + t5 * (t9 * t6 * (-t1 * t395 + t111 * t207) + t111 * t423 + t416 * t1);
	Jv[0][4] = -t113 * t458 + t436 * t95;
	t461 = t33 * t5;
	t463 = t27 * t97 + t461 * t6;
	t466 = t17 * t33;
	t467 = t27 * t7 + t466;
	t469 = t20 * t45;
	t471 = t37 * t467 - t469 * t6;
	t473 = t463 * t49 - t471 * t57;
	t474 = t473 * t111;
	t477 = t20 * t72;
	t478 = t37 * t6;
	t488 = t49 * t471 * t74 + t478 * t477 + t27 * (-t17 * t346 + t212 * t7) + (t17 * t212 + t346 * t7) * t33;
	t494 = t20 * t74;
	t509 = t17 * t461;
	t512 = t17 * t5;
	t514 = -t27 * t512 + t194;
	t517 = t17 * t469 + t37 * t514;
	t518 = t517 * t57;
	t532 = t49 * t517 * t74 - t366 * t477 + t27 * (-t212 * t512 - t346 * t6) + t33 * (t212 * t6 - t346 * t512);
	t551 = t11 * (t2 * (t1 * t488 + t474) + t52 * (t49 * t471 * t72 - t478 * t494 + t27 * (-t17 * t296 - t204 * t7) + t33 * (-t17 * t204 + t296 * t7))) + (t2 * (t111 * (t49 * (t27 * t30 - t509) - t518) + t1 * t532) + t52 * (t49 * t517 * t72 + t366 * t494 + t27 * (t204 * t512 - t296 * t6) - t33 * (t204 * t6 + t296 * t512))) * t9;
	t559 = t27 * t6 + t509;
	t561 = t49 * t559 + t518;
	t565 = t11 * (-t1 * t473 + t111 * t488) + (t1 * t561 + t111 * t532) * t9;
	Jv[0][5] = -t113 * t565 + t551 * t95;
	t568 = t3 * t74 + t124;
	t570 = t2 * t111;
	t572 = t49 * t568 - t57 * t570;
	t574 = t37 * t572 + t171;
	t577 = t49 * t570;
	t580 = t27 * t574 + (t568 * t57 + t577) * t33;
	t581 = dtey * t580;
	t584 = t45 * t572 + t165;
	t588 = dtey * t20;
	t589 = t11 * t584;
	t591 = t580 * dtex;
	t598 = t27 * (-t568 * t57 - t577) + t33 * t574;
	t607 = dtex * t20;
	t614 = t13 * (t5 * (dtex * t584 + t11 * t581) - t589 * t588 + t20 * t591 + t9 * t598 * dtey) + (t5 * (-dtey * t584 + t11 * t591) - t589 * t607 - t20 * t581 + t9 * t598 * dtex) * t15;
	t616 = -t23;
	t619 = t17 * t20;
	t620 = t11 * t5 * t616 - t619;
	t622 = t9 * t616;
	t624 = t27 * t620 + t33 * t622;
	t627 = t42 * t616 + t512;
	t628 = t45 * t627;
	t642 = t49 * t74 * (t37 * t624 - t628) - t74 * t57 * (t27 * t622 - t33 * t620) + t72 * (t37 * t627 + t45 * t624);
	t653 = t49 * (t23 * t27 * t9 + t33 * t620) - t37 * t57 * t624 + t57 * t628;
	Jv[0][6] = t1 * t113 * t653 - t111 * t113 * t642 - t614 * t95;
	Jv[1][0] = t112 * t95 + t113 * t94;
	Jv[1][1] = t113 * t140 + t152 * t95;
	Jv[1][2] = t1 * t271 * t95 - t111 * t239 * t95 + t113 * t201;
	Jv[1][3] = t113 * t325 - t336 * t95;
	Jv[1][4] = t113 * t436 + t458 * t95;
	Jv[1][5] = t113 * t551 + t565 * t95;
	Jv[1][6] = -t1 * t653 * t95 + t111 * t642 * t95 - t113 * t614;
	Jv[2][0] = t52 * t1 * t110 + t2 * (t104 * t74 + t108 * t72);
	t693 = -d3 - d4;
	t696 = t45 * (t41 + t43);
	Jv[2][1] = t52 * (t57 * (t1 * t20 * t204 * t6 * t9 - t1 * t35 * t37 * t74 - t111 * t25 * t33 + t111 * t27 * t32 + t111 * t693 + t120 * t696) + t49 * (-t1 * t27 * t32 * t74 + t111 * t469 * t6 * t9 - t111 * t35 * t37 + t25 * t74 * t87 + t111 * t696 - t120 * t693)) - t72 * (t102 * t49 + t131 * t57) * t2;
	t731 = -t9 * t467 + t27 * (-t19 + t24) + (t99 + d5 + d6) * t33;
	Jv[2][2] = t52 * (t45 * (t1 * (t72 * (t5 * t616 - t40 - t43) + t731 * t210) - t111 * t57 * t731) - t37 * (t1 * (t210 * t44 + t72 * t731) - t111 * t57 * t44)) - t2 * (t45 * (t49 * t72 * t731 + t44 * t74) + t37 * (-t44 * t49 * t72 + t731 * t74));
	t762 = t278 * t49 - t285;
	t769 = t303 * t49 + t309;
	Jv[2][3] = t52 * (t27 * (t1 * (-t74 * t762 - t288) + t281) - (t1 * (t74 * t769 + t312) + t306) * t33) + (t27 * (t72 * t762 - t295) + (t72 * t769 - t319) * t33) * t2;
	t786 = t338 * t49 + t355;
	t795 = t17 * t45 * t365;
	t801 = t11 * t466 * t57 + t369 * t49;
	t813 = t18 * t37 - t23 * t251;
	t817 = t23 * t355 + t414 * t49;
	Jv[2][4] = t52 * (t20 * (-t9 * t6 * (t1 * (t74 * t786 + t345) + t342) + t1 * (t72 * (t37 * t616 - t795) - t74 * t801) - t111 * t445) + t5 * (-t9 * t398 * t6 + t1 * (t72 * t813 - t74 * t817) - t417)) + (t20 * (t9 * (t72 * t786 - t354) * t6 + t72 * t801 - t74 * (t23 * t37 + t795)) + (t403 * t6 * t9 + t72 * t817 + t74 * t813) * t5) * t2;
	t845 = t37 * t39 + t45 * t467;
	t849 = t463 * t57 + t471 * t49;
	t857 = -t37 * t619 + t45 * t514;
	t861 = t49 * t517 - t559 * t57;
	Jv[2][5] = t52 * (t11 * (t1 * (t72 * t845 + t74 * t849) + t474) + (t1 * (t72 * t857 + t74 * t861) - t561 * t111) * t9) - t2 * (t11 * (t72 * t849 - t74 * t845) + (t72 * t861 - t74 * t857) * t9);
	t882 = dtex * t5 - dtey * t42;
	t884 = t11 * t5;
	t886 = dtey * t884 + t607;
	t889 = dtey * t249 + t27 * t886;
	t891 = t37 * t882 - t45 * t889;
	t895 = t37 * t889 + t45 * t882;
	t897 = t9 * t27;
	t900 = dtey * t897 - t33 * t886;
	t902 = t49 * t895 - t57 * t900;
	t914 = dtex * t42 + dtey * t5;
	t917 = dtex * t884 - t588;
	t920 = dtex * t249 + t27 * t917;
	t922 = t37 * t914 + t45 * t920;
	t926 = t37 * t920 - t45 * t914;
	t930 = dtex * t897 - t917 * t33;
	t932 = t49 * t926 - t57 * t930;
	Jv[2][6] = t52 * (t13 * (t1 * (t72 * t891 - t74 * t902) + t111 * (t49 * t900 + t57 * t895)) - t15 * (t1 * (t72 * t922 + t74 * t932) - t111 * (t49 * t930 + t57 * t926))) + (t13 * (t72 * t902 + t74 * t891) + t15 * (t72 * t932 - t74 * t922)) * t2;

	cartesian_velocity[0] = 0;
	cartesian_velocity[1] = 0;
	cartesian_velocity[2] = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < ROBOT_NUM_AXIS-ROBOT_NUM_MOTORCONTROLLER; j++) {
			cartesian_velocity[i] += Jv[i][j] * qp[j];
		}
	}
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withLinAxes\hom_transform_endeffector.hpp
void Robot::endeffectorPose9DOF(double *q, double *position, double *orientation) {
	double dwx0x = limits->world_param[0];
	double dwx0y = limits->world_param[1];
	double dwx0z = limits->world_param[2];
	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];
	
	double dtex = limits->tool_tcp[0];
	double dtey = limits->tool_tcp[1];
	double dtez = limits->tool_tcp[2];
	double rteyaw = limits->tool_tcp[3];
	double rtepitch = limits->tool_tcp[4];
	double rteroll = limits->tool_tcp[5];

	double d1 = 0.1575;
	double d2 = 0.2025;
	double d3 = 0.2375;
	double d4 = 0.1825;
	double d5 = 0.2175;
	double d6 = 0.1825;
	double d7 = 0.081;
	double d8 = 0.071;

	double dxx = 0;
	double dyy = 0;

	double Twe[4][4];
	double t1;
	double t10;
	double t101;
	double t105;
	double t109;
	double t110;
	double t112;
	double t12;
	double t121;
	double t124;
	double t126;
	double t129;
	double t13;
	double t133;
	double t134;
	double t138;
	double t139;
	double t14;
	double t143;
	double t145;
	double t150;
	double t152;
	double t153;
	double t158;
	double t16;
	double t160;
	double t164;
	double t166;
	double t169;
	double t17;
	double t172;
	double t174;
	double t176;
	double t178;
	double t18;
	double t182;
	double t184;
	double t188;
	double t189;
	double t190;
	double t191;
	double t193;
	double t198;
	double t199;
	double t2;
	double t20;
	double t203;
	double t208;
	double t21;
	double t212;
	double t214;
	double t216;
	double t22;
	double t220;
	double t221;
	double t223;
	double t224;
	double t226;
	double t237;
	double t24;
	double t240;
	double t244;
	double t246;
	double t248;
	double t25;
	double t250;
	double t267;
	double t271;
	double t276;
	double t28;
	double t280;
	double t286;
	double t288;
	double t290;
	double t292;
	double t299;
	double t3;
	double t30;
	double t301;
	double t303;
	double t305;
	double t308;
	double t31;
	double t310;
	double t312;
	double t313;
	double t314;
	double t315;
	double t316;
	double t317;
	double t32;
	double t321;
	double t323;
	double t325;
	double t331;
	double t334;
	double t335;
	double t34;
	double t340;
	double t344;
	double t348;
	double t353;
	double t356;
	double t366;
	double t367;
	double t379;
	double t38;
	double t381;
	double t383;
	double t385;
	double t389;
	double t391;
	double t395;
	double t396;
	double t397;
	double t398;
	double t4;
	double t40;
	double t400;
	double t404;
	double t408;
	double t41;
	double t413;
	double t417;
	double t42;
	double t421;
	double t425;
	double t44;
	double t443;
	double t445;
	double t465;
	double t469;
	double t47;
	double t49;
	double t492;
	double t495;
	double t497;
	double t499;
	double t5;
	double t50;
	double t503;
	double t505;
	double t506;
	double t512;
	double t513;
	double t52;
	double t521;
	double t522;
	double t525;
	double t527;
	double t532;
	double t549;
	double t55;
	double t57;
	double t570;
	double t579;
	double t58;
	double t6;
	double t62;
	double t63;
	double t66;
	double t68;
	double t7;
	double t71;
	double t75;
	double t77;
	double t80;
	double t82;
	double t84;
	double t87;
	double t89;
	double t9;
	double t93;
	double t95;
	double t99;
	Twe[0][0] = 0;
	Twe[0][1] = 0;
	Twe[0][2] = 0;
	Twe[0][3] = 0;
	Twe[1][0] = 0;
	Twe[1][1] = 0;
	Twe[1][2] = 0;
	Twe[1][3] = 0;
	Twe[2][0] = 0;
	Twe[2][1] = 0;
	Twe[2][2] = 0;
	Twe[2][3] = 0;
	Twe[3][0] = 0;
	Twe[3][1] = 0;
	Twe[3][2] = 0;
	Twe[3][3] = 0;
	t1 = q[6];
	t2 = cos_(t1);
	t3 = q[7];
	t4 = cos_(t3);
	t5 = t4 * t2;
	t6 = q[8];
	t7 = cos_(t6);
	t9 = sin_(t1);
	t10 = sin_(t6);
	t12 = -t9 * t10 + t5 * t7;
	t13 = q[5];
	t14 = cos_(t13);
	t16 = sin_(t13);
	t17 = t16 * t7;
	t18 = sin_(t3);
	t20 = t12 * t14 + t17 * t18;
	t21 = q[4];
	t22 = cos_(t21);
	t24 = sin_(t21);
	t25 = t7 * t4;
	t28 = t10 * t2 + t25 * t9;
	t30 = t20 * t22 - t24 * t28;
	t31 = q[3];
	t32 = cos_(t31);
	t34 = sin_(t31);
	t38 = t14 * t18 * t7 - t12 * t16;
	t40 = t30 * t32 - t34 * t38;
	t41 = q[2];
	t42 = cos_(t41);
	t44 = sin_(t41);
	t47 = t20 * t24 + t22 * t28;
	t49 = t40 * t42 - t44 * t47;
	t50 = sin_(roll);
	t52 = cos_(roll);
	t55 = t30 * t34 + t32 * t38;
	t57 = t49 * t50 + t52 * t55;
	t58 = sin_(pitch);
	t62 = t40 * t44 + t42 * t47;
	t63 = cos_(pitch);
	t66 = cos_(rteyaw);
	t68 = sin_(rteyaw);
	t71 = t10 * t5 + t7 * t9;
	t75 = t10 * t16 * t18 + t14 * t71;
	t77 = t9 * t4;
	t80 = -t10 * t77 + t2 * t7;
	t82 = t22 * t75 + t24 * t80;
	t84 = t18 * t14;
	t87 = t10 * t84 - t16 * t71;
	t89 = t32 * t82 - t34 * t87;
	t93 = t22 * t80 - t24 * t75;
	t95 = t42 * t89 + t44 * t93;
	t99 = t32 * t87 + t34 * t82;
	t101 = t50 * t95 + t52 * t99;
	t105 = t42 * t93 - t44 * t89;
	t109 = t66 * (t57 * t58 + t62 * t63) - (t101 * t58 - t105 * t63) * t68;
	t110 = cos_(yaw);
	t112 = sin_(yaw);
	t121 = t66 * (t49 * t52 - t50 * t55) - (-t50 * t99 + t52 * t95) * t68;
	t124 = cos_(rtepitch);
	t126 = t16 * t2;
	t129 = -t126 * t18 - t14 * t4;
	t133 = t16 * t4;
	t134 = t14 * t18 * t2 - t133;
	t138 = -t18 * t24 * t9 + t134 * t22;
	t139 = t138 * t34;
	t143 = -t129;
	t145 = t138 * t32 + t143 * t34;
	t150 = t18 * t22 * t9 + t134 * t24;
	t152 = t145 * t42 - t150 * t44;
	t153 = t50 * t152;
	t158 = t145 * t44 + t150 * t42;
	t160 = t58 * (t52 * (t129 * t32 + t139) + t153) + t63 * t158;
	t164 = t143 * t32 - t139;
	t166 = t152 * t52 + t164 * t50;
	t169 = sin_(rtepitch);
	Twe[0][0] = t124 * (t110 * t109 - t112 * t121) - t169 * (t110 * t160 - t112 * t166);
	t172 = t58 * t110;
	t174 = -t112 * t52 + t172 * t50;
	t176 = t63 * t110;
	t178 = t174 * t42 + t176 * t44;
	t182 = t112 * t50 + t172 * t52;
	t184 = t178 * t32 + t182 * t34;
	t188 = -t174 * t44 + t176 * t42;
	t189 = t24 * t188;
	t190 = t184 * t22 + t189;
	t191 = t190 * t7;
	t193 = t190 * t10;
	t198 = t178 * t34 - t182 * t32;
	t199 = t198 * t18;
	t203 = t7 * t198;
	t208 = t184 * t24 - t188 * t22;
	t212 = t208 * t9;
	t214 = t10 * t198;
	t216 = t7 * t18;
	t220 = t14 * (t191 * t5 - t193 * t9 - t199 * t7) + t2 * (-t10 * t208 + t133 * t203) - t25 * t212 + t16 * (t190 * t216 - t214 * t9);
	t221 = cos_(rteroll);
	t223 = t169 * t66;
	t224 = sin_(rteroll);
	t226 = -t221 * t68 + t223 * t224;
	t237 = t4 * t10;
	t240 = t10 * t18;
	t244 = t14 * (-t10 * t199 + t191 * t9 + t193 * t5) + t2 * (t133 * t214 + t208 * t7) - t237 * t212 + t16 * (t190 * t240 + t203 * t9);
	t246 = t169 * t68;
	t248 = t221 * t66 + t224 * t246;
	t250 = t18 * t2;
	t267 = t18 * t9;
	t271 = t22 * (t14 * t184 * t250 - t16 * t184 * t4 + t18 * t188 * t9) + t14 * (t189 * t250 + t198 * t4) + t250 * t198 * t16 - t24 * (t16 * t188 * t4 + t184 * t267);
	Twe[0][1] = t124 * t224 * t271 + t220 * t226 - t244 * t248;
	t276 = t221 * t223 + t224 * t68;
	t280 = -t221 * t246 + t224 * t66;
	Twe[0][2] = t124 * t221 * t271 + t220 * t276 + t244 * t280;
	t286 = dtex * t7 - dtey * t10;
	t288 = -dtez - d7 - d8;
	t290 = t18 * t286 + t288 * t4 - d5 - d6;
	t292 = -t288;
	t299 = dtex * t10 + dtey * t7;
	t301 = -t18 * t2 * t292 - t2 * t286 * t4 + t299 * t9;
	t303 = t14 * t290 + t16 * t301 - d3 - d4;
	t305 = -t301;
	t308 = t14 * t305 + t16 * t290;
	t310 = t22 * t308 * t34;
	t312 = t18 * t292 * t9;
	t313 = t2 * t299;
	t314 = t286 * t77;
	t315 = t312 + t313 + t314;
	t316 = t24 * t315;
	t317 = t34 * t316;
	t321 = t22 * t308 - t316;
	t323 = t34 * t303;
	t325 = t42 * (t32 * t321 - t323);
	t331 = t44 * t290;
	t334 = q[1];
	t335 = -t14 * t24 * t305 * t44 - t16 * t24 * t331 - t22 * t315 * t44 + t325 + t334;
	t340 = t24 * t308;
	t344 = t32 * t44 * t321;
	t348 = t16 * t305;
	t353 = q[0];
	t356 = t58 * (t52 * (t303 * t32 - d1 - d2 - dxx - dyy + t310 - t317) + t50 * t335) + t63 * (t42 * (t22 * t315 + t340) + t344 - t14 * t34 * t331 + t348 * t34 * t44 + t34 * (d3 + d4) * t44 + t353);
	t366 = t32 * (t14 * (-t18 * t286 + t292 * t4 + d5 + d6) + t348 + d3 + d4);
	t367 = t366 - t310 + t317 + d1 + dxx + dyy + d2;
	Twe[0][3] = -t112 * t335 * t52 - t112 * t367 * t50 + t110 * t356 + dwx0x;
	Twe[1][0] = t124 * (t109 * t112 + t110 * t121) - t169 * (t110 * t166 + t112 * t160);
	t379 = t58 * t112;
	t381 = t110 * t52 + t379 * t50;
	t383 = t63 * t112;
	t385 = t381 * t42 + t383 * t44;
	t389 = t110 * t50 - t379 * t52;
	t391 = t32 * t385 - t34 * t389;
	t395 = -t381 * t44 + t383 * t42;
	t396 = t24 * t395;
	t397 = t22 * t391 + t396;
	t398 = t7 * t397;
	t400 = t10 * t397;
	t404 = t32 * t389 + t34 * t385;
	t408 = t4 * t404;
	t413 = -t22 * t395 + t24 * t391;
	t417 = t9 * t413;
	t421 = t397 * t18;
	t425 = t14 * (-t216 * t404 + t398 * t5 - t400 * t9) + t2 * (-t10 * t413 + t17 * t408) - t25 * t417 + t16 * (-t10 * t404 * t9 + t421 * t7);
	t443 = t14 * (-t240 * t404 + t398 * t9 + t400 * t5) + t2 * (t10 * t16 * t408 + t413 * t7) - t237 * t417 + t16 * (t404 * t7 * t9 + t10 * t421);
	t445 = t124 * t224;
	t465 = t22 * (-t16 * t391 * t4 + t18 * t395 * t9 + t2 * t391 * t84) + t14 * (t250 * t396 + t408) + t250 * t16 * t404 - t24 * (t16 * t395 * t4 + t18 * t391 * t9);
	Twe[1][1] = t226 * t425 - t248 * t443 + t445 * t465;
	t469 = t124 * t221;
	Twe[1][2] = t276 * t425 + t280 * t443 + t465 * t469;
	Twe[1][3] = t110 * t335 * t52 + t110 * t367 * t50 + t112 * t356 + dwx0y;
	Twe[2][0] = t124 * (t63 * (-t101 * t68 + t57 * t66) - (t105 * t68 + t62 * t66) * t58) - t169 * (t63 * (-t164 * t52 + t153) - t158 * t58);
	t492 = t50 * t63;
	t495 = t42 * t492 - t44 * t58;
	t497 = t52 * t63;
	t499 = t32 * t495 + t34 * t497;
	t503 = t42 * t58 + t44 * t492;
	t505 = t22 * t499 - t24 * t503;
	t506 = t2 * t505;
	t512 = -t32 * t497 + t34 * t495;
	t513 = t18 * t512;
	t521 = t22 * t503 + t24 * t499;
	t522 = t521 * t10;
	t525 = t7 * t521;
	t527 = t512 * t10;
	t532 = t14 * (-t10 * t505 * t9 + t25 * t506 - t513 * t7) + t2 * (t16 * t25 * t512 - t522) - t77 * t525 + (t216 * t505 - t527 * t9) * t16;
	t549 = t14 * (t505 * t7 * t9 - t18 * t527 + t237 * t506) + t2 * (t133 * t527 + t525) - t77 * t522 + t16 * (t512 * t7 * t9 + t240 * t505);
	t570 = t22 * (t14 * t250 * t499 - t16 * t4 * t499 - t267 * t503) + t14 * (-t18 * t2 * t24 * t503 + t4 * t512) + t126 * t513 - t24 * (-t16 * t4 * t503 + t267 * t499);
	Twe[2][1] = t226 * t532 - t248 * t549 + t445 * t570;
	Twe[2][2] = t276 * t532 + t280 * t549 + t469 * t570;
	t579 = t22 * (-t2 * t299 - t312 - t314) - t340;
	Twe[2][3] = t63 * (t50 * (t44 * t579 + t325 + t334) - t52 * (t366 - t310 + t312 * t24 * t34 + t34 * t24 * (t313 + t314) + d1 + dxx + dyy + d2)) + t58 * (t323 * t44 + t42 * t579 - t344 - t353) + dwx0z;
	Twe[3][0] = 0.0e0;
	Twe[3][1] = 0.0e0;
	Twe[3][2] = 0.0e0;
	Twe[3][3] = 0.1e1;

	position[0] = Twe[0][3];
	position[1] = Twe[1][3];
	position[2] = Twe[2][3];

	if (Twe[2][0] < 1) {
		if (Twe[2][0] > -1) {
			orientation[0] = atan2_(Twe[1][0], Twe[0][0]); //yaw
			orientation[1] = asin_(-Twe[2][0]); //pitch
			orientation[2] = atan2_(Twe[2][1], Twe[2][2]); //roll
		}
		else {
			orientation[0] = -atan2_(-Twe[1][2], Twe[1][1]);
			orientation[1] = PI/2;
			orientation[2] = 0;
		}
	}
	else {
		orientation[0] = atan2_(-Twe[1][2], Twe[1][1]);
		orientation[1] = -PI / 2;
		orientation[2] = 0;
	}
}

//Source:
//Repository: ssh://git@gitlab.tuwien.ac.at:822/e376-acin/cds/robotics/kukalbriiwa/kukalbriiwa_model.git
//Revision: 3a26d1596709a3cbff7dae19ea4d99808e45c891
//Author: Christian Hartl - Nesic <christian.hartl@tuwien.ac.at>
//Date: 11 / 07 / 2024 13 : 23 : 55
//File: \cpp\maple_generated\withoutLinAxes\hom_transform_endeffector.hpp
void Robot::endeffectorPose7DOF(double *q, double *position, double *orientation) {
	double dw0x = limits->world_param[0];
	double dw0y = limits->world_param[1];
	double dw0z = limits->world_param[2];
	double yaw = limits->world_param[3];
	double pitch = limits->world_param[4];
	double roll = limits->world_param[5];

	double dtex = limits->tool_tcp[0];
	double dtey = limits->tool_tcp[1];
	double dtez = limits->tool_tcp[2];
	double rteyaw = limits->tool_tcp[3];
	double rtepitch = limits->tool_tcp[4];
	double rteroll = limits->tool_tcp[5];

	double d1 = 0.1575;
	double d2 = 0.2025;
	double d3 = 0.2375;
	double d4 = 0.1825;
	double d5 = 0.2175;
	double d6 = 0.1825;
	double d7 = 0.081;
	double d8 = 0.071;

	double Twe[4][4];
	double t1;
	double t10;
	double t100;
	double t101;
	double t104;
	double t105;
	double t107;
	double t11;
	double t111;
	double t112;
	double t114;
	double t117;
	double t119;
	double t12;
	double t120;
	double t13;
	double t130;
	double t132;
	double t136;
	double t144;
	double t15;
	double t158;
	double t16;
	double t17;
	double t170;
	double t171;
	double t173;
	double t175;
	double t177;
	double t18;
	double t182;
	double t184;
	double t188;
	double t191;
	double t193;
	double t197;
	double t199;
	double t2;
	double t20;
	double t201;
	double t202;
	double t205;
	double t207;
	double t208;
	double t21;
	double t212;
	double t218;
	double t22;
	double t220;
	double t224;
	double t24;
	double t243;
	double t247;
	double t251;
	double t253;
	double t254;
	double t256;
	double t257;
	double t260;
	double t262;
	double t264;
	double t267;
	double t27;
	double t273;
	double t275;
	double t278;
	double t28;
	double t281;
	double t282;
	double t284;
	double t289;
	double t29;
	double t293;
	double t297;
	double t3;
	double t30;
	double t300;
	double t301;
	double t308;
	double t309;
	double t313;
	double t316;
	double t318;
	double t32;
	double t327;
	double t33;
	double t339;
	double t341;
	double t349;
	double t356;
	double t359;
	double t36;
	double t364;
	double t366;
	double t377;
	double t38;
	double t382;
	double t386;
	double t39;
	double t392;
	double t394;
	double t4;
	double t400;
	double t405;
	double t407;
	double t41;
	double t410;
	double t415;
	double t416;
	double t417;
	double t419;
	double t421;
	double t422;
	double t423;
	double t424;
	double t425;
	double t427;
	double t43;
	double t430;
	double t434;
	double t436;
	double t437;
	double t44;
	double t440;
	double t441;
	double t442;
	double t443;
	double t459;
	double t47;
	double t473;
	double t476;
	double t478;
	double t480;
	double t488;
	double t490;
	double t491;
	double t493;
	double t496;
	double t497;
	double t499;
	double t5;
	double t50;
	double t504;
	double t508;
	double t513;
	double t523;
	double t53;
	double t543;
	double t545;
	double t55;
	double t559;
	double t56;
	double t562;
	double t577;
	double t591;
	double t595;
	double t6;
	double t604;
	double t608;
	double t62;
	double t63;
	double t631;
	double t635;
	double t648;
	double t651;
	double t653;
	double t655;
	double t659;
	double t661;
	double t662;
	double t664;
	double t668;
	double t669;
	double t676;
	double t677;
	double t68;
	double t682;
	double t687;
	double t69;
	double t7;
	double t703;
	double t72;
	double t726;
	double t81;
	double t83;
	double t89;
	double t9;
	double t95;
	double t98;
	Twe[0][0] = 0;
	Twe[0][1] = 0;
	Twe[0][2] = 0;
	Twe[0][3] = 0;
	Twe[1][0] = 0;
	Twe[1][1] = 0;
	Twe[1][2] = 0;
	Twe[1][3] = 0;
	Twe[2][0] = 0;
	Twe[2][1] = 0;
	Twe[2][2] = 0;
	Twe[2][3] = 0;
	Twe[3][0] = 0;
	Twe[3][1] = 0;
	Twe[3][2] = 0;
	Twe[3][3] = 0;
	t1 = q[3];
	t2 = cos_(t1);
	t3 = q[6];
	t4 = cos_(t3);
	t5 = t4 * t2;
	t6 = q[5];
	t7 = sin_(t6);
	t9 = sin_(t1);
	t10 = q[4];
	t11 = cos_(t10);
	t12 = cos_(t6);
	t13 = t12 * t11;
	t15 = sin_(t10);
	t16 = sin_(t3);
	t17 = t16 * t15;
	t18 = t13 * t4 - t17;
	t20 = t18 * t9 - t5 * t7;
	t21 = q[1];
	t22 = cos_(t21);
	t24 = sin_(t21);
	t27 = t7 * t9 * t4;
	t28 = t18 * t2 + t27;
	t29 = q[2];
	t30 = cos_(t29);
	t32 = sin_(t29);
	t33 = t4 * t12;
	t36 = t11 * t16 + t15 * t33;
	t38 = t28 * t30 - t32 * t36;
	t39 = t38 * t24;
	t41 = cos_(roll);
	t43 = q[0];
	t44 = sin_(t43);
	t47 = cos_(t43);
	t50 = t11 * t47;
	t53 = t32 * t47;
	t55 = t24 * t44;
	t56 = t4 * t7;
	t62 = t4 * t47;
	t63 = t7 * t32;
	t68 = t22 * t38 * t44 + t30 * t36 * t47 + t2 * (t32 * t33 * t50 - t17 * t53 - t55 * t56) + (t11 * t33 * t55 - t17 * t55 + t62 * t63) * t9;
	t69 = sin_(roll);
	t72 = sin_(pitch);
	t81 = t7 * t24;
	t83 = t32 * t44;
	t89 = t24 * t47;
	t95 = cos_(pitch);
	t98 = cos_(rteyaw);
	t100 = sin_(rteyaw);
	t101 = t7 * t2;
	t104 = t15 * t4;
	t105 = t13 * t16 + t104;
	t107 = -t101 * t16 + t105 * t9;
	t111 = t16 * t7 * t9;
	t112 = t105 * t2 + t111;
	t114 = t15 * t12;
	t117 = t11 * t4 - t114 * t16;
	t119 = t112 * t30 + t117 * t32;
	t120 = t24 * t119;
	t130 = t15 * t32;
	t132 = t16 * t7;
	t136 = t44 * t12;
	t144 = -t22 * t44 * t119 + t30 * t117 * t47 + t2 * (-t12 * t16 * t32 * t50 - t130 * t62 + t132 * t55) - (t11 * t136 * t16 * t24 + t104 * t55 + t132 * t53) * t9;
	t158 = t24 * t12;
	t170 = t98 * (t72 * (t41 * (t20 * t22 - t39) + t69 * t68) + t95 * (t22 * t38 * t47 - t30 * t36 * t44 + t2 * (-t13 * t32 * t4 * t44 + t17 * t83 - t62 * t81) + (t24 * t33 * t50 - t89 * t17 - t56 * t83) * t9)) - (t72 * (t41 * (t107 * t22 - t120) - t144 * t69) + t95 * (t22 * t119 * t47 + t30 * t117 * t44 + t2 * (-t13 * t16 * t83 - t104 * t83 - t132 * t89) + (t15 * t24 * t62 + t158 * t16 * t50 - t132 * t83) * t9)) * t100;
	t171 = cos_(yaw);
	t173 = sin_(yaw);
	t175 = -t20;
	t177 = t175 * t22 + t39;
	t182 = -t107;
	t184 = t182 * t22 + t120;
	t188 = t98 * (t177 * t69 + t41 * t68) + t100 * (t144 * t41 - t184 * t69);
	t191 = cos_(rtepitch);
	t193 = sin_(rtepitch);
	t197 = t11 * t7 * t9 + t12 * t2;
	t199 = t11 * t2;
	t201 = t9 * t12;
	t202 = t199 * t7 - t201;
	t205 = -t130 * t7 + t202 * t30;
	t207 = t197 * t22 - t205 * t24;
	t208 = t207 * t41;
	t212 = t7 * t15;
	t218 = t11 * t7;
	t220 = t12 * t47;
	t224 = t22 * t44 * t205 + t212 * t30 * t47 + t2 * (t136 * t24 + t50 * t63) - (-t218 * t55 + t220 * t32) * t9;
	t243 = t72 * (t224 * t69 + t208) + t95 * (t22 * t47 * t205 - t212 * t44 * t30 + t2 * (-t11 * t44 * t63 + t220 * t24) + (t136 * t32 + t50 * t81) * t9);
	t247 = -t207 * t69 + t224 * t41;
	Twe[0][0] = t191 * (t170 * t171 - t173 * t188) - (t171 * t243 - t173 * t247) * t193;
	t251 = cos_(rteroll);
	t253 = t193 * t98;
	t254 = sin_(rteroll);
	t256 = -t100 * t251 + t253 * t254;
	t257 = t95 * t171;
	t260 = t72 * t171;
	t262 = -t173 * t41 + t260 * t69;
	t264 = -t257 * t44 + t262 * t47;
	t267 = t22 * t47;
	t273 = t173 * t69 + t260 * t41;
	t275 = t22 * t262 * t44 - t24 * t273 + t257 * t267;
	t278 = t257 * t89;
	t281 = t22 * t273;
	t282 = t24 * t262 * t44 + t278 + t281;
	t284 = t2 * t264 * t32 + t2 * t275 * t30 + t282 * t9;
	t289 = -t264 * t30 + t275 * t32;
	t293 = t15 * t289;
	t297 = t15 * t16 * t2 - t27;
	t300 = t16 * t275;
	t301 = t15 * t30;
	t308 = t4 * t275;
	t309 = t30 * t7;
	t313 = t11 * (t12 * t284 * t4 - t16 * t289) - t33 * t293 - t32 * t297 * t264 + t2 * (-t282 * t56 - t300 * t301) + t9 * (-t15 * t16 * t282 + t308 * t309);
	t316 = t193 * t100;
	t318 = t251 * t98 + t254 * t316;
	t327 = t15 * t5 + t111;
	t339 = t11 * (t12 * t16 * t284 + t289 * t4) - t12 * t16 * t293 + t32 * t327 * t264 + t2 * (-t132 * t282 + t301 * t308) + t9 * (t15 * t282 * t4 + t300 * t309);
	t341 = t7 * t275;
	t349 = t32 * t11;
	t356 = t262 * t7;
	t359 = t278 + t281;
	t364 = t32 * t95;
	t366 = t24 * t11;
	t377 = t30 * (-t12 * t275 * t9 + t15 * t264 * t7 + t199 * t341) + t2 * (t44 * (-t171 * t349 * t7 * t95 + t158 * t262) + t32 * t356 * t50 + t12 * t359) + t9 * (t44 * (t12 * t171 * t364 + t356 * t366) - t32 * t262 * t220 + t11 * t7 * t359) - t130 * t341;
	Twe[0][1] = t191 * t254 * t377 + t256 * t313 - t318 * t339;
	t382 = t100 * t254 + t251 * t253;
	t386 = -t251 * t316 + t254 * t98;
	Twe[0][2] = t191 * t251 * t377 + t313 * t382 + t339 * t386;
	t392 = -dtex * t4 + dtey * t16;
	t394 = dtez + d7 + d8;
	t400 = -t392;
	t405 = dtex * t16 + dtey * t4;
	t407 = t11 * t12 * t400 + t11 * t394 * t7 - t15 * t405;
	t410 = t22 * (t2 * (t12 * t394 + t392 * t7 + d5 + d6) + t9 * t407 + d3 + d4);
	t415 = -t12 * t394 + t400 * t7 - d5 - d6;
	t416 = t9 * t415;
	t417 = t2 * t407 + t416;
	t419 = t30 * t24 * t417;
	t421 = t7 * t394 * t15;
	t422 = t11 * t405;
	t423 = t400 * t114;
	t424 = t421 + t422 + t423;
	t425 = t32 * t424;
	t427 = t24 * t425 + d1 + d2 + t410 - t419;
	t430 = t30 * t417 - t425;
	t434 = -t407;
	t436 = t2 * t415 + t434 * t9 - d3 - d4;
	t437 = t24 * t436;
	t440 = t32 * t417;
	t441 = t30 * t424 + t440;
	t442 = t47 * t441;
	t443 = t22 * t430 * t44 - t437 * t44 + t442;
	t459 = t72 * (t41 * t427 + t443 * t69) + t95 * (t22 * t430 * t47 + t44 * (t30 * (-t11 * t405 - t421 - t423) - t440) - t24 * t436 * t47);
	Twe[0][3] = -t173 * t41 * t443 + t173 * t427 * t69 + t171 * t459 + dw0x;
	Twe[1][0] = t191 * (t170 * t173 + t171 * t188) - (t171 * t247 + t173 * t243) * t193;
	t473 = t95 * t173;
	t476 = t72 * t173;
	t478 = t171 * t41 + t476 * t69;
	t480 = -t44 * t473 + t47 * t478;
	t488 = t171 * t69 - t41 * t476;
	t490 = t22 * t44 * t478 + t24 * t488 + t267 * t473;
	t491 = t490 * t30;
	t493 = t473 * t89;
	t496 = t488 * t22;
	t497 = t24 * t44 * t478 + t493 - t496;
	t499 = t2 * t32 * t480 + t2 * t491 + t497 * t9;
	t504 = -t30 * t480 + t32 * t490;
	t508 = t12 * t504;
	t513 = t15 * t490;
	t523 = t11 * (t12 * t4 * t499 - t16 * t504) - t508 * t104 - t32 * t297 * t480 + t2 * (-t16 * t30 * t513 - t497 * t56) + (-t15 * t16 * t497 + t491 * t56) * t9;
	t543 = t11 * (t12 * t16 * t499 + t4 * t504) - t508 * t17 + t32 * t327 * t480 + t2 * (t30 * t4 * t513 - t132 * t497) + t9 * (t15 * t4 * t497 + t132 * t491);
	t545 = t7 * t490;
	t559 = t478 * t7;
	t562 = t493 - t496;
	t577 = t30 * (-t12 * t490 * t9 + t15 * t480 * t7 + t199 * t545) + t2 * (t44 * (-t173 * t349 * t7 * t95 + t158 * t478) + t32 * t559 * t50 + t562 * t12) + t9 * (t44 * (t12 * t173 * t364 + t366 * t559) - t32 * t478 * t220 + t562 * t218) - t130 * t545;
	Twe[1][1] = t191 * t254 * t577 + t256 * t523 - t318 * t543;
	Twe[1][2] = t191 * t251 * t577 + t382 * t523 + t386 * t543;
	Twe[1][3] = t171 * t41 * t443 - t171 * t427 * t69 + t173 * t459 + dw0y;
	t591 = t28 * t32 + t30 * t36;
	t595 = -t175 * t24 + t22 * t38;
	t604 = -t112 * t32 + t117 * t30;
	t608 = t119 * t22 - t182 * t24;
	t631 = -t202 * t32 - t301 * t7;
	t635 = t197 * t24 + t205 * t22;
	Twe[2][0] = t191 * (t95 * (t98 * (t69 * (t44 * t595 + t47 * t591) - t41 * t177) + t100 * (t69 * (-t44 * t608 + t47 * t604) + t184 * t41)) - t72 * (t98 * (-t44 * t591 + t47 * t595) - t100 * (t44 * t604 + t47 * t608))) + (t95 * (t69 * (-t44 * t635 + t47 * t631) - t208) + (t44 * t631 + t47 * t635) * t72) * t193;
	t648 = t69 * t95;
	t651 = -t44 * t648 + t47 * t72;
	t653 = t41 * t95;
	t655 = t22 * t651 + t24 * t653;
	t659 = t44 * t72 + t47 * t648;
	t661 = t30 * t655 - t32 * t659;
	t662 = t4 * t661;
	t664 = t16 * t661;
	t668 = -t22 * t653 + t24 * t651;
	t669 = t4 * t668;
	t676 = t30 * t659 + t32 * t655;
	t677 = t676 * t16;
	t682 = t668 * t16;
	t687 = t2 * (t13 * t662 - t15 * t664 - t669 * t7) + t11 * (t201 * t669 - t677) - t33 * t15 * t676 + t9 * (-t15 * t682 + t662 * t7);
	t703 = t2 * (t13 * t664 + t15 * t662 - t682 * t7) + t11 * (t201 * t682 + t4 * t676) - t114 * t677 + t9 * (t15 * t669 + t664 * t7);
	t726 = (t30 * (t101 * t11 * t655 - t12 * t655 * t9 - t212 * t659) + t2 * (-t11 * t63 * t659 + t12 * t668) + t218 * t668 * t9 - t32 * (-t12 * t659 * t9 + t15 * t655 * t7)) * t191;
	Twe[2][1] = -t254 * t726 - t256 * t687 + t318 * t703;
	Twe[2][2] = -t251 * t726 - t382 * t687 - t386 * t703;
	Twe[2][3] = t95 * (t69 * (t442 + (t22 * t430 - t437) * t44) + (t410 - t419 + t421 * t32 * t24 + t24 * t32 * (t422 + t423) + d1 + d2) * t41) + t72 * (t47 * (t22 * (t30 * (t2 * t434 - t416) + t425) + t437) + t441 * t44) + dw0z;
	Twe[3][0] = 0.0e0;
	Twe[3][1] = 0.0e0;
	Twe[3][2] = 0.0e0;
	Twe[3][3] = 0.1e1;

	position[0] = Twe[0][3];
	position[1] = Twe[1][3];
	position[2] = Twe[2][3];

	if (Twe[2][0] < 1) {
		if (Twe[2][0] > -1) {
			orientation[0] = atan2_(Twe[1][0], Twe[0][0]); //yaw
			orientation[1] = asin_(-Twe[2][0]); //pitch
			orientation[2] = atan2_(Twe[2][1], Twe[2][2]); //roll
		}
		else {
			orientation[0] = -atan2_(-Twe[1][2], Twe[1][1]);
			orientation[1] = PI / 2;
			orientation[2] = 0;
		}
	}
	else {
		orientation[0] = atan2_(-Twe[1][2], Twe[1][1]);
		orientation[1] = -PI / 2;
		orientation[2] = 0;
	}
}

Event* Robot::getEvent() {
	return event_handler->getEvent();
}
