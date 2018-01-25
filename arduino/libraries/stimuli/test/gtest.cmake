
set(GOOGLETEST_ROOT gtest/googletest CACHE STRING "Google Test source root")

# SYSTEM keyword is to suppress some build warnings, to not get too many 
# notifications from Travis CI (how?)
# TODO why also prepend PROJECT_SOURCE_DIR? so include is useful higher up?
include_directories(SYSTEM
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/include
)

set(GOOGLETEST_SOURCES
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest_main.cc
)
# was it actually conventional style to have the close paren at the same tab?

foreach(_source ${GOOGLETEST_SOURCES})
    set_source_files_properties(${_source} PROPERTIES GENERATED 1)
endforeach()

# TODO stuff to check out gtest submodule during build?
# already doing that somewhere?

add_library(gtest ${GOOGLETEST_SOURCES})

