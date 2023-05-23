cmake_minimum_required (VERSION 3.4)
project (VCU ASM C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(STLIB_DIR /opt/malva/ST-LIB)
set(EXECUTABLE ${PROJECT_NAME}.elf)

file(GLOB_RECURSE SOURCE_C ${CMAKE_SOURCE_DIR}/Core/*.c ${CMAKE_SOURCE_DIR}/Drivers/*.c ${CMAKE_SOURCE_DIR}/Middlewares/*.c ${CMAKE_SOURCE_DIR}/LWIP/*.c) 
file(GLOB_RECURSE SOURCE_CPP ${CMAKE_SOURCE_DIR}/Core/*.cpp ${CMAKE_SOURCE_DIR}/Drivers/*.cpp ${CMAKE_SOURCE_DIR}/Middlewares/*.cpp ${CMAKE_SOURCE_DIR}/LWIP/*.cpp) 
file(GLOB_RECURSE SOURCE_H ${CMAKE_SOURCE_DIR}/Core/*.h ${CMAKE_SOURCE_DIR}/Drivers/*.h ${CMAKE_SOURCE_DIR}/Middlewares/*.h ${CMAKE_SOURCE_DIR}/LWIP/*.h) 
file(GLOB_RECURSE SOURCE_HPP ${CMAKE_SOURCE_DIR}/Core/*.hpp ${CMAKE_SOURCE_DIR}/Drivers/*.hpp ${CMAKE_SOURCE_DIR}/Middlewares/*.hpp ${CMAKE_SOURCE_DIR}/LWIP/*.hpp) 

add_subdirectory(${STLIB_DIR} ${STLIB_DIR}/build)
add_executable(${EXECUTABLE} 
  ${SOURCE_C}
  ${SOURCE_CPP}
  ${SOURCE_H}
  ${SOURCE_HPP}
  Core/Startup/startup_stm32h723zgtx.s
  )

target_link_libraries(${EXECUTABLE} st-lib)

target_compile_definitions(${EXECUTABLE} PUBLIC
  -DUSE_HAL_DRIVER
  -DSTM32H723xx
  -DDATA_IN_D2_SRAM
)

target_include_directories(${EXECUTABLE} PUBLIC
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/CMSIS/Include
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  Middlewares/Third_Party/LwIP/src/include
  Middlewares/Third_Party/LwIP/system
  Middlewares/Third_Party/LwIP/src/netif/ppp
  Middlewares/Third_Party/LwIP/src/include/lwip
  Middlewares/Third_Party/LwIP/src/include/lwip/apps
  Middlewares/Third_Party/LwIP/src/include/lwip/priv
  Middlewares/Third_Party/LwIP/src/include/lwip/prot
  Middlewares/Third_Party/LwIP/src/include/netif
  Middlewares/Third_Party/LwIP/src/include/compat
  Middlewares/Third_Party/LwIP/src/include/compat/posix
  Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa
  Middlewares/Third_Party/LwIP/src/include/compat/posix/net
  Middlewares/Third_Party/LwIP/src/include/compat/posix/sys
  Middlewares/Third_Party/LwIP/src/include/compat/stdc
  Middlewares/Third_Party/LwIP/system/arch
  Drivers/BSP/Components
  Drivers/BSP/Components/lan8742

  Core/Inc
)


target_compile_options(${EXECUTABLE} PUBLIC
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  
  ${PROFILE}
  ${OPTIMIZATION}
  -ffunction-sections
  -fdata-sections
  -fno-exceptions
  -fno-use-cxa-atexit
  -fno-use-cxa-atexit

  -Wall

  --specs=nosys.specs
)
target_link_options(${EXECUTABLE} PUBLIC 
        -T${CMAKE_SOURCE_DIR}/STM32H723ZGTX_FLASH.ld
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -specs=nosys.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )
