///////////////////////////////////////////////////////////////////////////////
// IiwaSimCtrl.h

#ifndef __IIWASIMCTRL_H__
#define __IIWASIMCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "IiwaSimW32.h"
#include "TcBase.h"
#include "IiwaSimClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CIiwaSimCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CIiwaSimCtrl, &CLSID_IiwaSimCtrl>
	, public IIiwaSimCtrl
	, public ITcOCFCtrlImpl<CIiwaSimCtrl, CIiwaSimClassFactory>
{
public:
	CIiwaSimCtrl();
	virtual ~CIiwaSimCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_IIWASIMCTRL)
DECLARE_NOT_AGGREGATABLE(CIiwaSimCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CIiwaSimCtrl)
	COM_INTERFACE_ENTRY(IIiwaSimCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __IIWASIMCTRL_H__
