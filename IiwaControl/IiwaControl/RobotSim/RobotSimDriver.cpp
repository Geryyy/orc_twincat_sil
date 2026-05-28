///////////////////////////////////////////////////////////////////////////////
// RobotSimDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RobotSimDriver.h"
#include "RobotSimClassFactory.h"

DECLARE_GENERIC_DEVICE(ROBOTSIMDRV)

IOSTATUS CRobotSimDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRobotSimClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRobotSimDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRobotSimDriver::ROBOTSIMDRV_GetVersion( )
{
	return( (ROBOTSIMDRV_Major << 8) | ROBOTSIMDRV_Minor );
}

