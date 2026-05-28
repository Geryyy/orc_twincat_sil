///////////////////////////////////////////////////////////////////////////////
// IiwaSimDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "IiwaSimDriver.h"
#include "IiwaSimClassFactory.h"

DECLARE_GENERIC_DEVICE(IIWASIMDRV)

IOSTATUS CIiwaSimDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CIiwaSimClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CIiwaSimDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CIiwaSimDriver::IIWASIMDRV_GetVersion( )
{
	return( (IIWASIMDRV_Major << 8) | IIWASIMDRV_Minor );
}

