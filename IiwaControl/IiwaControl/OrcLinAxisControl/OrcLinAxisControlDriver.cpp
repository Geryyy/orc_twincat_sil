///////////////////////////////////////////////////////////////////////////////
// OrcLinAxisControlDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "OrcLinAxisControlDriver.h"
#include "OrcLinAxisControlClassFactory.h"

DECLARE_GENERIC_DEVICE(ORCLINAXISCONTROLDRV)

IOSTATUS COrcLinAxisControlDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new COrcLinAxisControlClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID COrcLinAxisControlDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl COrcLinAxisControlDriver::ORCLINAXISCONTROLDRV_GetVersion( )
{
	return( (ORCLINAXISCONTROLDRV_Major << 8) | ORCLINAXISCONTROLDRV_Minor );
}

