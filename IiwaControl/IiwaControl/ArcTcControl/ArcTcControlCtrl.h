///////////////////////////////////////////////////////////////////////////////
// ArcTcControlCtrl.h

#ifndef __ARCTCCONTROLCTRL_H__
#define __ARCTCCONTROLCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "ArcTcControlW32.h"
#include "TcBase.h"
#include "ArcTcControlClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CArcTcControlCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CArcTcControlCtrl, &CLSID_ArcTcControlCtrl>
	, public IArcTcControlCtrl
	, public ITcOCFCtrlImpl<CArcTcControlCtrl, CArcTcControlClassFactory>
{
public:
	CArcTcControlCtrl();
	virtual ~CArcTcControlCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ARCTCCONTROLCTRL)
DECLARE_NOT_AGGREGATABLE(CArcTcControlCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CArcTcControlCtrl)
	COM_INTERFACE_ENTRY(IArcTcControlCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ARCTCCONTROLCTRL_H__
