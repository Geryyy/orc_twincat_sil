// ArcTcControlCtrl.cpp : Implementation of CTcArcTcControlCtrl
#include "TcPch.h"
#pragma hdrstop

#include "ArcTcControlW32.h"
#include "ArcTcControlCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CArcTcControlCtrl

CArcTcControlCtrl::CArcTcControlCtrl() 
	: ITcOCFCtrlImpl<CArcTcControlCtrl, CArcTcControlClassFactory>() 
{
}

CArcTcControlCtrl::~CArcTcControlCtrl()
{
}

