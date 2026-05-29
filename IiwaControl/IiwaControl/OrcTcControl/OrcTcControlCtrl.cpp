// OrcTcControlCtrl.cpp : Implementation of CTcOrcTcControlCtrl
#include "TcPch.h"
#pragma hdrstop

#include "OrcTcControlW32.h"
#include "OrcTcControlCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// COrcTcControlCtrl

COrcTcControlCtrl::COrcTcControlCtrl() 
	: ITcOCFCtrlImpl<COrcTcControlCtrl, COrcTcControlClassFactory>() 
{
}

COrcTcControlCtrl::~COrcTcControlCtrl()
{
}

