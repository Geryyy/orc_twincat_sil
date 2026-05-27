///////////////////////////////////////////////////////////////////////////////
// ArcLinAxisControlDriver.h

#ifndef __ARCLINAXISCONTROLDRIVER_H__
#define __ARCLINAXISCONTROLDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ARCLINAXISCONTROLDRV_NAME        "ARCLINAXISCONTROL"
#define ARCLINAXISCONTROLDRV_Major       1
#define ARCLINAXISCONTROLDRV_Minor       0

#define DEVICE_CLASS CArcLinAxisControlDriver

#include "ObjDriver.h"

class CArcLinAxisControlDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ARCLINAXISCONTROLDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ARCLINAXISCONTROLDRV)
	VxD_Service( ARCLINAXISCONTROLDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ARCLINAXISCONTROLDRIVER_H__