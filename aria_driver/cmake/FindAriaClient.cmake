set(FIND_ARIACLIENT_PATHS ${PROJECT_SOURCE_DIR}/libraries/AriaClient)

find_path(ARIACLIENT_INCLUDE_DIR AriaClient.h
        PATH_SUFFIXES include
        PATHS ${FIND_ARIACLIENT_PATHS})

find_library(ARIACLIENT_LIBRARY
        NAMES libAriaClient.a
        PATH_SUFFIXES lib
        PATHS ${FIND_ARIACLIENT_PATHS})