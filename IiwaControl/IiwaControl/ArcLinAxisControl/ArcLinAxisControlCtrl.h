///////////////////////////////////////////////////////////////////////////////
// ArcLinAxisControlCtrl.h

#ifndef __ARCLINAXISCONTROLCTRL_H__
#define __ARCLINAXISCONTROLCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "ArcLinAxisControlW32.h"
#include "TcBase.h"
#include "ArcLinAxisControlClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CArcLinAxisControlCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CArcLinAxisControlCtrl, &CLSID_ArcLinAxisControlCtrl>
	, public IArcLinAxisControlCtrl
	, public ITcOCFCtrlImpl<CArcLinAxisControlCtrl, CArcLinAxisControlClassFactory>
{
public:
	CArcLinAxisControlCtrl();
	virtual ~CArcLinAxisControlCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ARCLINAXISCONTROLCTRL)
DECLARE_NOT_AGGREGATABLE(CArcLinAxisControlCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CArcLinAxisControlCtrl)
	COM_INTERFACE_ENTRY(IArcLinAxisControlCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ARCLINAXISCONTROLCTRL_H__
