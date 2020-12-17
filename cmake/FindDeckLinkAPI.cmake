find_library(
    DeckLinkAPI_LIBRARIES
    NAMES DeckLinkAPI
    PATHS /usr/lib /usr/local/lib
    REQUIRED 
)

find_path(
    DeckLinkAPI_INCLUDE_DIRS
    NAMES DeckLinkAPI.h
    PATHS ${DECKLINK_SDK_DIR}/include
    REQUIRED
)

if (NOT (DeckLinkAPI_LIBRARIES AND DeckLinkAPI_INCLUDE_DIRS))
    message(FATAL_ERROR "Couldn't find DeckLinkAPI, make sure to set DECKLINK_SDK_DIR.")
endif(NOT (DeckLinkAPI_LIBRARIES AND DeckLinkAPI_INCLUDE_DIRS))
