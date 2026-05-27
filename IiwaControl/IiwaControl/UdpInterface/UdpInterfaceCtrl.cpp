// UdpInterfaceCtrl.cpp : Implementation of CTcUdpInterfaceCtrl
#include "TcPch.h"
#pragma hdrstop

#include "UdpInterfaceW32.h"
#include "UdpInterfaceCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CUdpInterfaceCtrl

CUdpInterfaceCtrl::CUdpInterfaceCtrl() 
	: ITcOCFCtrlImpl<CUdpInterfaceCtrl, CUdpInterfaceClassFactory>() 
{
}

CUdpInterfaceCtrl::~CUdpInterfaceCtrl()
{
}

