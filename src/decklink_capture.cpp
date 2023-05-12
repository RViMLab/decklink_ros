#include "decklink_capture.hpp"

DeckLinkCapture::DeckLinkCapture() : refCount_(0), frame_count_(0), callback_(0) {}

ULONG DeckLinkCapture::AddRef(void) { return __sync_add_and_fetch(&refCount_, 1); }

ULONG DeckLinkCapture::Release(void) {
  int32_t newRefValue = __sync_sub_and_fetch(&refCount_, 1);
  if (newRefValue == 0) {
    delete this;
    return 0;
  }
  return newRefValue;
}

HRESULT DeckLinkCapture::VideoInputFrameArrived(IDeckLinkVideoInputFrame *video_frame,
                                                IDeckLinkAudioInputPacket *) {
  if (!video_frame) {
    return S_OK;
  }

  frame_count_++;

  if (video_frame->GetFlags() & bmdFrameHasNoInputSource) {
    fprintf(stderr, "Frame %lu: No input signal\n", frame_count_);
  }

  if (!callback_) {
    fprintf(stderr, "Frame %lu: No callback registered, dropping frame\n", frame_count_);
    return S_OK;
  } else {
    void *raw_frame;
    video_frame->GetBytes(&raw_frame);
    callback_(frame_count_, raw_frame, video_frame->GetWidth(), video_frame->GetHeight(),
              video_frame->GetRowBytes() / video_frame->GetWidth());
  }

  return S_OK;
}

bool DeckLinkCapture::init() {
  IDeckLinkIterator *deckLinkIterator = CreateDeckLinkIteratorInstance();
  IDeckLink *deckLink;

  if (deckLinkIterator->Next(&deckLink) != S_OK) {
    fprintf(stderr, "DeckLinkCapture::init(): Failed to get DeckLink device.\n");
    return false;
  }

  IDeckLinkInput *deckLinkInput;
  if (deckLink->QueryInterface(IID_IDeckLinkInput, (void **)&deckLinkInput) != S_OK) {
    fprintf(stderr, "DeckLinkCapture::init(): Failed to get input from device.\n");
    return false;
  }

  if (deckLinkInput->SetCallback(this) != S_OK) {
    fprintf(stderr, "DeckLinkCapture::init(): Failed to set callback.\n");
    return false;
  }

  BMDDisplayMode display_mode = bmdModeHD1080p5994; // possibly search for formats here
  BMDPixelFormat pixel_format = bmdFormat8BitYUV;

  if (deckLinkInput->EnableVideoInput(display_mode, pixel_format, 0) != S_OK) {
    fprintf(stderr, "DeckLinkCapture::init(): Failed to enable video input.\n");
    return false;
  }

  if (deckLinkInput->StartStreams() != S_OK) {
    fprintf(stderr, "DeckLinkCapture::init(): Failed to start streams.\n");
    return false;
  }

  return true;
}
