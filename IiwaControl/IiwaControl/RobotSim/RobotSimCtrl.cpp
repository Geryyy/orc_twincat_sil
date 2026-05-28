// RobotSimCtrl.cpp : Implementation of CTcRobotSimCtrl
#include "TcPch.h"
#pragma hdrstop

#include "RobotSimW32.h"
#include "RobotSimCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CRobotSimCtrl

CRobotSimCtrl::CRobotSimCtrl() 
	: ITcOCFCtrlImpl<CRobotSimCtrl, CRobotSimClassFactory>() 
{
}

CRobotSimCtrl::~CRobotSimCtrl()
{
}

