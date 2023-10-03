cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME "BACnetServerExamplePico")

set(PICO_BOARD pico_w)


set(LIB_STACK "CASBACnetStack_x64_Release")
set(LIB_PATH "bin/CASBACnetStack_x64_Release.a")

SET (CC "g++")

#set(CFLAGS "-m64 -Wall")
set(CFLAGS "-Wall")

set(DEBUGFLAGS "-O0 -g3 -DCAS_BACNET_STACK_LIB_TYPE_LIB")
set(RELEASEFLAGS "-DCAS_BACNET_STACK_LIB_TYPE_LIB")
#set(OBJECTFLAGS "-c -fmessage-length=0 -fPIC -MMD -MP")
#set(OBJECTFLAGS "-c -fPIC -MMD -MP")
set(LDFLAGS "-static")

#add_compile_options(${RELEASEFLAGS} ${CFLAGS} ${OBJECTFLAGS} ${INCLUDES})
#add_compile_options(-Wall -DCAS_BACNET_STACK_LIB_TYPE_LIB -fexceptions)
#SET(CMAKE_CXX_FLAGS "-Wall -DCAS_BACNET_STACK_LIB_TYPE_LIB -fmessage-length=0 -fPIC -MMD -MP -fexceptions" )
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
#target_compile_definitions(${PROJECT_NAME}
#    PUBLIC O3 CAS_BACNET_STACK_LIB_TYPE_LIB
#)

set(SOURCES 
    "submodules/cas-bacnet-stack/source/*.cpp"
    "BACnetServerExamplePico.cpp"
    "submodules/cas-bacnet-stack/adapters/cpp/*.cpp"
    "submodules/cas-bacnet-stack/submodules/cas-common/source/*.cpp"
    )

set(INCLUDES
    "bin"
    "submodules/cas-bacnet-stack/adapters/cpp"
    "submodules/cas-bacnet-stack/source"
    "submodules/cas-bacnet-stack/submodules/cas-common/source"
    "submodules/cas-bacnet-stack/submodules/xml2json/include"
    )  


# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(${PROJECT_NAME})
    
#include directories
include_directories(
    ${INCLUDES}
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

# include files
file(GLOB SRC_FILES
    #"submodules/cas-bacnet-stack/source/*.cpp"
    #"submodules/cas-bacnet-stack/source/*.h"
    ${SOURCES}
)

# build the chipkin bacnet stack
#add_library(${STACK_LIBRARY} STATIC ${LIB_SRC_FILES})

# rest of your project
add_executable(${PROJECT_NAME}
    BACnetServerExamplePico.cpp
    ${SRC_FILES}
)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background 
    hardware_adc
    )

# Enable USB serial debugging and disable UART
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
