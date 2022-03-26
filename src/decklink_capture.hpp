#pragma once

#include <functional>
#include <DeckLinkAPI.h>


class DeckLinkCapture : public IDeckLinkInputCallback {
    public:
        typedef std::function<void (long unsigned frame_count, void* raw_frame, unsigned raw_frame_width, unsigned raw_frame_height, unsigned bytes_per_pixel)> DeckLinkCaptureCallbackType;

        DeckLinkCapture();

        virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, LPVOID*) { return E_NOINTERFACE; };
        virtual ULONG STDMETHODCALLTYPE AddRef(void);
        virtual ULONG STDMETHODCALLTYPE  Release(void);
        virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags) { return S_OK; };
        virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

        bool init();
        void setCallback(DeckLinkCaptureCallbackType callback) { callback_ = callback; };

    private:
        int32_t	refCount_;
        unsigned long frame_count_;

        DeckLinkCaptureCallbackType callback_;
};
