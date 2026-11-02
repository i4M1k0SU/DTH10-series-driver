// Host-side check of the hand-made MSVC RTTI (feature `rtti-check`, see
// build.rs): the object from CreateBonDriver must survive a real
// `dynamic_cast<IBonDriver2*>` by the MSVC runtime, and the IBonDriver2
// vtable slots must land on the right functions. The class declarations
// mirror TVTest's IBonDriver2.h (Unicode build: LPCTSTR = const wchar_t*).
#include <windows.h>

class IBonDriver {
public:
    virtual const BOOL OpenTuner(void) = 0;
    virtual void CloseTuner(void) = 0;
    virtual const BOOL SetChannel(const BYTE bCh) = 0;
    virtual const float GetSignalLevel(void) = 0;
    virtual const DWORD WaitTsStream(const DWORD dwTimeOut = 0) = 0;
    virtual const DWORD GetReadyCount(void) = 0;
    virtual const BOOL GetTsStream(BYTE *pDst, DWORD *pdwSize, DWORD *pdwRemain) = 0;
    virtual const BOOL GetTsStream(BYTE **ppDst, DWORD *pdwSize, DWORD *pdwRemain) = 0;
    virtual void PurgeTsStream(void) = 0;
    virtual void Release(void) = 0;
};

class IBonDriver2 : public IBonDriver {
public:
    virtual LPCWSTR GetTunerName(void) = 0;
    virtual const BOOL IsTunerOpening(void) = 0;
    virtual LPCWSTR EnumTuningSpace(const DWORD dwSpace) = 0;
    virtual LPCWSTR EnumChannelName(const DWORD dwSpace, const DWORD dwChannel) = 0;
    virtual const BOOL SetChannel(const DWORD dwSpace, const DWORD dwChannel) = 0;
    virtual const DWORD GetCurSpace(void) = 0;
    virtual const DWORD GetCurChannel(void) = 0;
};

extern "C" IBonDriver *CreateBonDriver(void);

// 0 = everything as TVTest expects; otherwise the number of the first failed step.
extern "C" int dth10_rtti_check(void) {
    IBonDriver *p = CreateBonDriver();
    if (p == nullptr) {
        return 1;
    }
    IBonDriver2 *p2 = dynamic_cast<IBonDriver2 *>(p);
    if (p2 == nullptr) {
        p->Release();
        return 2;
    }
    int failed = 0;
    if (p2->GetTunerName() == nullptr) {
        failed = 3;
    } else if (p2->EnumTuningSpace(0) == nullptr) {
        failed = 4;
    } else if (p2->EnumTuningSpace(99) != nullptr) {
        failed = 5;
    } else if (p2->IsTunerOpening() != FALSE) {
        failed = 6;
    } else if (p2->GetReadyCount() != 0) {
        failed = 7;
    } else if (p2->GetCurSpace() != 0 || p2->GetCurChannel() != 0) {
        failed = 8;
    }
    p2->Release();
    return failed;
}
