///////////////////////////////////////////////////////////////////////////////
// UdpInterfaceCtrl.h

#ifndef __UDPINTERFACECTRL_H__
#define __UDPINTERFACECTRL_H__

#include <atlbase.h>
#include <atlcom.h>

#define UDPINTERFACEDRV_NAME "UDPINTERFACE"

#include "resource.h"       // main symbols
#include "UdpInterfaceW32.h"
#include "TcBase.h"
#include "UdpInterfaceClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CUdpInterfaceCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CUdpInterfaceCtrl, &CLSID_UdpInterfaceCtrl>
	, public IUdpInterfaceCtrl
	, public ITcOCFCtrlImpl<CUdpInterfaceCtrl, CUdpInterfaceClassFactory>
{
public:
	CUdpInterfaceCtrl();
	virtual ~CUdpInterfaceCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_UDPINTERFACECTRL)
DECLARE_NOT_AGGREGATABLE(CUdpInterfaceCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CUdpInterfaceCtrl)
	COM_INTERFACE_ENTRY(IUdpInterfaceCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __UDPINTERFACECTRL_H__
