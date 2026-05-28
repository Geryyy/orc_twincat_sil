///////////////////////////////////////////////////////////////////////////////
// IiwaSimDriver.h

#ifndef __IIWASIMDRIVER_H__
#define __IIWASIMDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define IIWASIMDRV_NAME        "IIWASIM"
#define IIWASIMDRV_Major       1
#define IIWASIMDRV_Minor       0

#define DEVICE_CLASS CIiwaSimDriver

#include "ObjDriver.h"

class CIiwaSimDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl IIWASIMDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(IIWASIMDRV)
	VxD_Service( IIWASIMDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __IIWASIMDRIVER_H__