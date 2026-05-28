///////////////////////////////////////////////////////////////////////////////
// RobotSimDriver.h

#ifndef __ROBOTSIMDRIVER_H__
#define __ROBOTSIMDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ROBOTSIMDRV_NAME        "ROBOTSIM"
#define ROBOTSIMDRV_Major       1
#define ROBOTSIMDRV_Minor       0

#define DEVICE_CLASS CRobotSimDriver

#include "ObjDriver.h"

class CRobotSimDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ROBOTSIMDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ROBOTSIMDRV)
	VxD_Service( ROBOTSIMDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ROBOTSIMDRIVER_H__