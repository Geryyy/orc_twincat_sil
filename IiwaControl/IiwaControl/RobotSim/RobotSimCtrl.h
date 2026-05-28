///////////////////////////////////////////////////////////////////////////////
// RobotSimCtrl.h

#ifndef __ROBOTSIMCTRL_H__
#define __ROBOTSIMCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "RobotSimW32.h"
#include "TcBase.h"
#include "RobotSimClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CRobotSimCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CRobotSimCtrl, &CLSID_RobotSimCtrl>
	, public IRobotSimCtrl
	, public ITcOCFCtrlImpl<CRobotSimCtrl, CRobotSimClassFactory>
{
public:
	CRobotSimCtrl();
	virtual ~CRobotSimCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ROBOTSIMCTRL)
DECLARE_NOT_AGGREGATABLE(CRobotSimCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CRobotSimCtrl)
	COM_INTERFACE_ENTRY(IRobotSimCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ROBOTSIMCTRL_H__
