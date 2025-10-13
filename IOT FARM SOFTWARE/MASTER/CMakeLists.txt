cmake_minimum_required(VERSION 3.5...3.27)
set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
# Set the full path to your Arduino toolchain
set(ARDUINO_TOOLCHAIN_DIR "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin")

# Force AVR GCC compiler with full paths
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER "${ARDUINO_TOOLCHAIN_DIR}/avr-gcc.exe")
set(CMAKE_CXX_COMPILER "${ARDUINO_TOOLCHAIN_DIR}/avr-g++.exe")
set(CMAKE_ASM_COMPILER "${ARDUINO_TOOLCHAIN_DIR}/avr-gcc.exe")
set(CMAKE_SYSTEM_PROCESSOR avr)

# Must set these BEFORE the project() command
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)  # Disable executable tests

# AVR Configuration - MUST BE SET BEFORE PROJECT()
set(MCU atmega328p)
set(F_CPU 8000000UL)
set(BAUD 19200)

# Add compiler flags for AVR - MUST BE SET BEFORE PROJECT()
set(CMAKE_C_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -D__AVR_ATmega328P__ -Os -Wall" CACHE STRING "C Compiler Flags")
set(CMAKE_CXX_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -D__AVR_ATmega328P__ -Os -Wall" CACHE STRING "C++ Compiler Flags")
set(CMAKE_ASM_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -D__AVR_ATmega328P__ -x assembler-with-cpp" CACHE STRING "ASM Compiler Flags")
set(CMAKE_EXE_LINKER_FLAGS "-mmcu=${MCU}" CACHE STRING "Linker Flags")

# add common C++ standard
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

project(IOTFARM C CXX ASM)

# Disable all compiler checks
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER_WORKS 1 CACHE INTERNAL "")

# Arduino core and variant & spi
set(ARDUINO_CORE_PATH "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino")
set(ARDUINO_VARIANT_PATH "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.6/variants/standard")
set(ARDUINO_SPI_PATH "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/SPI/src")
set(ARDUINO_SOFTWARE_SERIAL_PATH "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/SoftwareSerial/src")
# set(ARDUINO_TINYGPSPLUS_PATH "C:/Users/SoftClansUser/Documents/Arduino/libraries/TinyGPSPlus/src")

#
# Set source files with full absolute paths
# set(ARDUINO_RTCLIB_SOURCES "${PROJECT_BASE_DIR}/RTC_DS3231.cpp"
#     "${PROJECT_BASE_DIR}/RTC_I2C.cpp"
#     "${PROJECT_BASE_DIR}/RTClib.cpp"
#     )
# set(ARDUINO_SD_SOURCES 
#     "${PROJECT_BASE_DIR}/SD.cpp"
#     "${PROJECT_BASE_DIR}/utility/Sd2Card.cpp"
#     "${PROJECT_BASE_DIR}/utility/SdFile.cpp" 
#     "${PROJECT_BASE_DIR}/utility/SdVolume.cpp"
# )
set(ARDUINO_WIRE_SOURCES 
    "${PROJECT_BASE_DIR}/Wire.cpp"
    "${PROJECT_BASE_DIR}/utility/twi.c"
)
set(ADAFRUIT_BUSIO_SOURCES ${PROJECT_BASE_DIR}/Adafruit_I2CDevice.cpp)
set(ARDUINO_SD_FILE_SRC "${PROJECT_BASE_DIR}/File.cpp")

# Add Arduino core and variant includes
include_directories(
    ${ARDUINO_CORE_PATH}
    ${ARDUINO_VARIANT_PATH}
    ${ARDUINO_SPI_PATH}
    ${ARDUINO_SOFTWARE_SERIAL_PATH}
   # ${ARDUINO_TINYGPSPLUS_PATH}
    ${PROJECT_BASE_DIR}/utility 
)

# Grab all Arduino core source files (excluding USB files if not needed)
file(GLOB ARDUINO_CORE_SOURCES
    "${ARDUINO_CORE_PATH}/*.c"
    "${ARDUINO_CORE_PATH}/*.cpp"
    "${ARDUINO_CORE_PATH}/*.S"
    "${ARDUINO_SPI_PATH}/*.cpp"
    # Exclude USB files if not using USB
    #"${ARDUINO_CORE_PATH}/USBCore.cpp"
    #"${ARDUINO_CORE_PATH}/CDC.cpp"
    #"${ARDUINO_CORE_PATH}/PluggableUSB.cpp"
)

file(GLOB ARDUINO_SOFTWARE_SERIAL_SOURCES "${ARDUINO_SOFTWARE_SERIAL_PATH}/*.cpp")
#file(GLOB ARDUINO_TINYGPSPLUS_SOURCES "${ARDUINO_TINYGPSPLUS_PATH}/*.cpp")

# Add source files - include your sketch (blink.cpp)
add_executable(
    ${PROJECT_NAME} 
    ${ARDUINO_CORE_SOURCES}
    ${ARDUINO_SOFTWARE_SERIAL_SOURCES}
    ${ARDUINO_WIRE_SOURCES}
   # ${ARDUINO_RTCLIB_SOURCES}
   # ${ARDUINO_SD_SOURCES}
  #  ${ADAFRUIT_BUSIO_SOURCES}
 #   ${ARDUINO_SD_FILE_SRC}
    "${PROJECT_BASE_DIR}/blink.cpp"   # <- add your sketch
)

# Additional includes - use project target name not NODE
target_include_directories(${PROJECT_NAME} PRIVATE .)

# Use toolchain objcopy/size if available
find_program(AVR_OBJCOPY "${ARDUINO_TOOLCHAIN_DIR}/avr-objcopy.exe" HINTS "${ARDUINO_TOOLCHAIN_DIR}")
if(NOT AVR_OBJCOPY)
    find_program(AVR_OBJCOPY avr-objcopy)
endif()
find_program(AVR_SIZE "${ARDUINO_TOOLCHAIN_DIR}/avr-size.exe" HINTS "${ARDUINO_TOOLCHAIN_DIR}")
if(NOT AVR_SIZE)
    find_program(AVR_SIZE avr-size)
endif()

# Create HEX file and print size
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND "${AVR_OBJCOPY}" -O ihex -R .eeprom $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND "${AVR_SIZE}" --format=avr --mcu=${MCU} $<TARGET_FILE:${PROJECT_NAME}>
)

# Flash target - adjust COM port and programmer as needed
# Default uses avrdude from PATH and Arduino bootloader protocol (-c arduino).
add_custom_target(flash
    COMMAND avrdude -C "C:/Users/SoftClansUser/AppData/Local/Arduino15/packages/arduino/tools/avr/etc/avrdude.conf" -c arduino -p m328p -P COM3 -b ${BAUD} -U flash:w:${PROJECT_NAME}.hex:i
    DEPENDS ${PROJECT_NAME}.hex
    COMMENT "Flashing ${PROJECT_NAME}.hex to device (adjust COM port/programmer if needed)"
)