///////////////////////////////////////////////////////////////////////////////
// ArcTcControlDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "ArcTcControlDriver.h"
#include "ArcTcControlClassFactory.h"

DECLARE_GENERIC_DEVICE(ARCTCCONTROLDRV)

IOSTATUS CArcTcControlDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CArcTcControlClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CArcTcControlDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CArcTcControlDriver::ARCTCCONTROLDRV_GetVersion( )
{
	return( (ARCTCCONTROLDRV_Major << 8) | ARCTCCONTROLDRV_Minor );
}

