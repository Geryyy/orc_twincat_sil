///////////////////////////////////////////////////////////////////////////////
// OrcTcControlDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "OrcTcControlDriver.h"
#include "OrcTcControlClassFactory.h"

DECLARE_GENERIC_DEVICE(ORCTCCONTROLDRV)

IOSTATUS COrcTcControlDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new COrcTcControlClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID COrcTcControlDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl COrcTcControlDriver::ORCTCCONTROLDRV_GetVersion( )
{
	return( (ORCTCCONTROLDRV_Major << 8) | ORCTCCONTROLDRV_Minor );
}

