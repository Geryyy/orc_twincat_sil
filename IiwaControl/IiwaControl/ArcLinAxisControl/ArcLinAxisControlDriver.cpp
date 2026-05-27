///////////////////////////////////////////////////////////////////////////////
// ArcLinAxisControlDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "ArcLinAxisControlDriver.h"
#include "ArcLinAxisControlClassFactory.h"

DECLARE_GENERIC_DEVICE(ARCLINAXISCONTROLDRV)

IOSTATUS CArcLinAxisControlDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CArcLinAxisControlClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CArcLinAxisControlDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CArcLinAxisControlDriver::ARCLINAXISCONTROLDRV_GetVersion( )
{
	return( (ARCLINAXISCONTROLDRV_Major << 8) | ARCLINAXISCONTROLDRV_Minor );
}

