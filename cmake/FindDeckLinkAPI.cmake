find_library(
    DeckLinkAPI_LIBRARY
    NAMES DeckLinkAPI
    PATHS /usr/lib /usr/local/lib
    REQUIRED 
)

find_path(
    DeckLinkAPI_INCLUDE_DIR
    NAMES DeckLinkAPI.h
    PATHS ${DECKLINK_SDK_DIR}/include
    REQUIRED
)

if (NOT (DeckLinkAPI_LIBRARY AND DeckLinkAPI_INCLUDE_DIR))
    message(FATAL_ERROR "Couldn't find DeckLinkAPI, make sure to set DECKLINK_SDK_DIR.")
endif(NOT (DeckLinkAPI_LIBRARY AND DeckLinkAPI_INCLUDE_DIR))
