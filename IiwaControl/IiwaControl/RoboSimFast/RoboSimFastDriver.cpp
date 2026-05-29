///////////////////////////////////////////////////////////////////////////////
// RoboSimFastDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RoboSimFastDriver.h"
#include "RoboSimFastClassFactory.h"

DECLARE_GENERIC_DEVICE(ROBOSIMFASTDRV)

IOSTATUS CRoboSimFastDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRoboSimFastClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRoboSimFastDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRoboSimFastDriver::ROBOSIMFASTDRV_GetVersion( )
{
	return( (ROBOSIMFASTDRV_Major << 8) | ROBOSIMFASTDRV_Minor );
}

