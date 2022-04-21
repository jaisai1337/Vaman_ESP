# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sridhar/esp/esp-idf-v4.3/components/bootloader/subproject"
  "/home/sridhar/esp/Vaman_spi/build/bootloader"
  "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix"
  "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix/tmp"
  "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix/src/bootloader-stamp"
  "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix/src"
  "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sridhar/esp/Vaman_spi/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
