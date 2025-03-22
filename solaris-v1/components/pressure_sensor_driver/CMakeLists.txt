idf_component_register(SRCS "bmp390.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver general)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../general/include)
                        