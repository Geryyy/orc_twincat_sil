///////////////////////////////////////////////////////////////////////////////
// OrcTcControlDriver.h

#ifndef __ORCTCCONTROLDRIVER_H__
#define __ORCTCCONTROLDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ORCTCCONTROLDRV_NAME        "ORCTCCONTROL"
#define ORCTCCONTROLDRV_Major       1
#define ORCTCCONTROLDRV_Minor       0

#define DEVICE_CLASS COrcTcControlDriver

#include "ObjDriver.h"

class COrcTcControlDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ORCTCCONTROLDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ORCTCCONTROLDRV)
	VxD_Service( ORCTCCONTROLDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ORCTCCONTROLDRIVER_H__