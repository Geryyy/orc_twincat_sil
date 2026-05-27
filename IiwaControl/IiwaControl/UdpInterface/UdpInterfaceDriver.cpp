///////////////////////////////////////////////////////////////////////////////
// UdpInterfaceDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "UdpInterfaceDriver.h"
#include "UdpInterfaceClassFactory.h"

DECLARE_GENERIC_DEVICE(UDPINTERFACEDRV)

IOSTATUS CUdpInterfaceDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CUdpInterfaceClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CUdpInterfaceDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CUdpInterfaceDriver::UDPINTERFACEDRV_GetVersion( )
{
	return( (UDPINTERFACEDRV_Major << 8) | UDPINTERFACEDRV_Minor );
}

