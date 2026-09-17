# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/restops/esp/esp-idf/components/bootloader/subproject"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/tmp"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/src/bootloader-stamp"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/src"
  "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/restops/UMD-Loop-Phase-II-Technology-Challenge/E1/system_four/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
