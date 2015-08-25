cmake_minimum_required(VERSION 2.8)
project(h2sl_cdcg)
set(CDCG_VERSION 1.0)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)

# add the include directories to the build
set(DIRS symbol
         llm
         dcg
         util
          )
include_directories(${DIRS} ${CMAKE_INSTALL_PREFIX}/include ${CMAKE_BINARY_DIR}/include)

# search for the gengetopt program, exit if not found
find_program(GENGETOPT gengetopt)
if( NOT GENGETOPT )
  message( FATAL_ERROR "gengetopt not found\n" )
  return()
endif()

# function to generate the gengetopt source and header from a .ggo file
function(generate_ggo) 
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  cmake_parse_arguments(GENERATE_GGO "" "INPUT;OUTPUT" "" ${ARGN})
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_GGO_OUTPUT}.c ${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_GGO_OUTPUT}.h
    COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_GGO_INPUT} --file-name ${GENERATE_GGO_OUTPUT} --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts
    DEPDENDS ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_GGO_INPUT}
  )
endfunction(generate_ggo)

# search for the libxml2 package, exit if not found
find_package(LibXml2)
if( NOT LIBXML2_FOUND )
  message( FATAL_ERROR "libxml2 not found\n" )
  return()
endif()
include_directories(${LIBXML2_INCLUDE_DIR})

# search for the boost package, exit if not found
find_package(Boost COMPONENTS system thread filesystem)
if( NOT Boost_FOUND )
  message( FATAL_ERROR "Boost not found\n" )
  return()
endif()
include_directories(${Boost_INCLUDE_DIR})

# search for the LBFGS libraries
find_library(LBFGS_LIBRARY NAMES liblbfgs lbfgs HINT ${CMAKE_BINARY_DIR}/lib /usr/lib /usr/local/lib /opt/local/lib)
if( NOT LBFGS_LIBRARY )
  message( FATAL_ERROR "lbfgs not found\n")
  return()
endif()

# search for the QT package, exit if not found
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
if( NOT Qt4_FOUND )
  message( FATAL_ERROR "Qt4 not found\n" )
  return()
endif()
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})

find_package(h2sl)
if( NOT h2sl_FOUND )
  message( FATAL_ERROR "h2sl not found\n")
  return()
endif()
include_directories(${H2SL_INCLUDE_DIR})
#message("H2SL_INCLUDE_DIR:${H2SL_INCLUDE_DIR}")
#message("H2SL_LIBRARIES:${H2SL_LIBRARIES}")

# function for installing headers
function(process_hdrs)
  cmake_parse_arguments(PROCESS_HDRS "" "" "HDRS;QT_HDRS" ${ARGN})
  foreach(msg_hdr ${MSG_HDRS})
    set(msg_hdr_fullpath "${CMAKE_BINARY_DIR}/${msg_hdr}")
    set(MSG_HDRS_FULLPATH ${MSG_HDRS_FULLPATH} ${msg_hdr_fullpath})
  endforeach(msg_hdr)
  install(FILES
          ${HDRS}
          ${QT_HDRS}
          ${MSG_HDRS_FULLPATH}
          DESTINATION include/h2sl_cdcg)
endfunction(process_hdrs)

# function for compiling and installing a library
function(process_lib)
  cmake_parse_arguments(PROCESS_LIB "" "LIB" "SRCS;QT_HDRS;DEPS" ${ARGN})
  if( QT_HDRS )
    qt_wrap_cpp(${LIB} MOC_SRCS ${QT_HDRS})
  endif( QT_HDRS )
  add_library(${PROCESS_LIB_LIB} SHARED
              ${PROCESS_LIB_SRCS}
              ${MOC_SRCS} 
              ${MSG_HDRS})
  target_link_libraries(${PROCESS_LIB_LIB} ${PROCESS_LIB_DEPS})
  install(TARGETS ${PROCESS_LIB_LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
endfunction(process_lib)

# function for compiling and installing the binaries for the h2sl project
function(process_bin_srcs)
  cmake_parse_arguments(PROCESS_BIN_SRCS "" "LIB" "BIN_SRCS;GGOS;DEPS" ${ARGN})
  foreach(ggo ${PROCESS_BIN_SRCS_GGOS})
    string(REGEX REPLACE ".ggo" "_cmdline" tmp ${ggo})
    generate_ggo(INPUT ${ggo} OUTPUT ${tmp})
  endforeach(ggo)
  foreach(bin_src ${PROCESS_BIN_SRCS_BIN_SRCS})
    set(bin_src_cmdline "${CMAKE_CURRENT_BINARY_DIR}/${bin_src}")
    string(REGEX REPLACE ".cc" "_cmdline.c" bin_src_cmdline ${bin_src_cmdline})
    set(bin "h2sl-${bin_src}")
    string(REGEX REPLACE ".cc" "" bin ${bin})
    string(REGEX REPLACE "_" "-" bin ${bin})
    add_executable(${bin} ${bin_src} ${bin_src_cmdline})
    target_link_libraries(${bin} ${PROCESS_BIN_SRCS_LIB} ${PROCESS_BIN_SRCS_DEPS})
    set(BINS ${BINS} ${bin})
  endforeach(bin_src)
  install(TARGETS
          ${BINS}
          DESTINATION bin)
endfunction(process_bin_srcs)

# add the source directories
foreach(dir ${DIRS})
  add_subdirectory(${dir})
endforeach(dir ${DIRS})

