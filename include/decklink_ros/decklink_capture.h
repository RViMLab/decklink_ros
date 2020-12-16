#pragma once

#include <sensor_msgs/Image.h>
#include <DeckLinkAPI.h>


class DeckLinkCapture : public IDeckLinkInputCallback {
    public:
        DeckLinkCapture() = default;

        virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
        virtual ULONG STDMETHODCALLTYPE AddRef(void);
        virtual ULONG STDMETHODCALLTYPE  Release(void);
        virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
        virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

    private:
        int32_t				refCount_;
};
