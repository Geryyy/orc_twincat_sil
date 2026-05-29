///////////////////////////////////////////////////////////////////////////////
// OrcLinAxisControlCtrl.h

#ifndef __ORCLINAXISCONTROLCTRL_H__
#define __ORCLINAXISCONTROLCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "OrcLinAxisControlW32.h"
#include "TcBase.h"
#include "OrcLinAxisControlClassFactory.h"
#include "TcOCFCtrlImpl.h"

class COrcLinAxisControlCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<COrcLinAxisControlCtrl, &CLSID_OrcLinAxisControlCtrl>
	, public IOrcLinAxisControlCtrl
	, public ITcOCFCtrlImpl<COrcLinAxisControlCtrl, COrcLinAxisControlClassFactory>
{
public:
	COrcLinAxisControlCtrl();
	virtual ~COrcLinAxisControlCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ORCLINAXISCONTROLCTRL)
DECLARE_NOT_AGGREGATABLE(COrcLinAxisControlCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(COrcLinAxisControlCtrl)
	COM_INTERFACE_ENTRY(IOrcLinAxisControlCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ORCLINAXISCONTROLCTRL_H__
