///////////////////////////////////////////////////////////////////////////////
// UdpInterfaceDriver.h

#ifndef __UDPINTERFACEDRIVER_H__
#define __UDPINTERFACEDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define UDPINTERFACEDRV_NAME        "UDPINTERFACE"
#define UDPINTERFACEDRV_Major       1
#define UDPINTERFACEDRV_Minor       0

#define DEVICE_CLASS CUdpInterfaceDriver

#include "ObjDriver.h"

class CUdpInterfaceDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl UDPINTERFACEDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(UDPINTERFACEDRV)
	VxD_Service( UDPINTERFACEDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __UDPINTERFACEDRIVER_H__