///////////////////////////////////////////////////////////////////////////////
// OrcLinAxisControlDriver.h

#ifndef __ORCLINAXISCONTROLDRIVER_H__
#define __ORCLINAXISCONTROLDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ORCLINAXISCONTROLDRV_NAME        "ORCLINAXISCONTROL"
#define ORCLINAXISCONTROLDRV_Major       1
#define ORCLINAXISCONTROLDRV_Minor       0

#define DEVICE_CLASS COrcLinAxisControlDriver

#include "ObjDriver.h"

class COrcLinAxisControlDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ORCLINAXISCONTROLDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ORCLINAXISCONTROLDRV)
	VxD_Service( ORCLINAXISCONTROLDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ORCLINAXISCONTROLDRIVER_H__