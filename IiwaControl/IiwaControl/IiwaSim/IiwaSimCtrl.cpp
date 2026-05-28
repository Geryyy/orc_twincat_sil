// IiwaSimCtrl.cpp : Implementation of CTcIiwaSimCtrl
#include "TcPch.h"
#pragma hdrstop

#include "IiwaSimW32.h"
#include "IiwaSimCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CIiwaSimCtrl

CIiwaSimCtrl::CIiwaSimCtrl() 
	: ITcOCFCtrlImpl<CIiwaSimCtrl, CIiwaSimClassFactory>() 
{
}

CIiwaSimCtrl::~CIiwaSimCtrl()
{
}

