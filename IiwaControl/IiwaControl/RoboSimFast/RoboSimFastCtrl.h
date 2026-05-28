///////////////////////////////////////////////////////////////////////////////
// RoboSimFastCtrl.h

#ifndef __ROBOSIMFASTCTRL_H__
#define __ROBOSIMFASTCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "RoboSimFastW32.h"
#include "TcBase.h"
#include "RoboSimFastClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CRoboSimFastCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CRoboSimFastCtrl, &CLSID_RoboSimFastCtrl>
	, public IRoboSimFastCtrl
	, public ITcOCFCtrlImpl<CRoboSimFastCtrl, CRoboSimFastClassFactory>
{
public:
	CRoboSimFastCtrl();
	virtual ~CRoboSimFastCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ROBOSIMFASTCTRL)
DECLARE_NOT_AGGREGATABLE(CRoboSimFastCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CRoboSimFastCtrl)
	COM_INTERFACE_ENTRY(IRoboSimFastCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ROBOSIMFASTCTRL_H__
