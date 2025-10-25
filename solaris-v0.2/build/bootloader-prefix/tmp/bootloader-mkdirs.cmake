# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/opt/esp/idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "/opt/esp/idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader"
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix"
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/tmp"
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/src/bootloader-stamp"
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/src"
  "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspaces/software-solaris/solaris-v0.2/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
