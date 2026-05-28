///////////////////////////////////////////////////////////////////////////////
// RoboSimFastDriver.h

#ifndef __ROBOSIMFASTDRIVER_H__
#define __ROBOSIMFASTDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ROBOSIMFASTDRV_NAME        "ROBOSIMFAST"
#define ROBOSIMFASTDRV_Major       1
#define ROBOSIMFASTDRV_Minor       0

#define DEVICE_CLASS CRoboSimFastDriver

#include "ObjDriver.h"

class CRoboSimFastDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ROBOSIMFASTDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ROBOSIMFASTDRV)
	VxD_Service( ROBOSIMFASTDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ROBOSIMFASTDRIVER_H__