///////////////////////////////////////////////////////////////////////////////
// ArcTcControlDriver.h

#ifndef __ARCTCCONTROLDRIVER_H__
#define __ARCTCCONTROLDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ARCTCCONTROLDRV_NAME        "ARCTCCONTROL"
#define ARCTCCONTROLDRV_Major       1
#define ARCTCCONTROLDRV_Minor       0

#define DEVICE_CLASS CArcTcControlDriver

#include "ObjDriver.h"

class CArcTcControlDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ARCTCCONTROLDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ARCTCCONTROLDRV)
	VxD_Service( ARCTCCONTROLDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ARCTCCONTROLDRIVER_H__