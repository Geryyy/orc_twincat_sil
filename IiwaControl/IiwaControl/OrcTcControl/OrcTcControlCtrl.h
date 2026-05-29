///////////////////////////////////////////////////////////////////////////////
// OrcTcControlCtrl.h

#ifndef __ORCTCCONTROLCTRL_H__
#define __ORCTCCONTROLCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "OrcTcControlW32.h"
#include "TcBase.h"
#include "OrcTcControlClassFactory.h"
#include "TcOCFCtrlImpl.h"

class COrcTcControlCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<COrcTcControlCtrl, &CLSID_OrcTcControlCtrl>
	, public IOrcTcControlCtrl
	, public ITcOCFCtrlImpl<COrcTcControlCtrl, COrcTcControlClassFactory>
{
public:
	COrcTcControlCtrl();
	virtual ~COrcTcControlCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ORCTCCONTROLCTRL)
DECLARE_NOT_AGGREGATABLE(COrcTcControlCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(COrcTcControlCtrl)
	COM_INTERFACE_ENTRY(IOrcTcControlCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ORCTCCONTROLCTRL_H__
