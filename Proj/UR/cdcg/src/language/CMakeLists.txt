# GENGETOPT FILES
set(GGOS
   )

# HEADER FILES
set(HDRS
    h2sl_cdcg/phrase.h
   )

# QT HEADER FILES
set(QT_HDRS )

# SOURCE FILES
set(SRCS
    phrase.cc
   )

# BINARY SOURCE FILES
set(BIN_SRCS
   )

# LIBRARY DEPENDENCIES
set(DEPS ${H2SL_LIBRARIES} ${LIBXML2_LIBRARIES})

# LIBRARY NAME
set(LIB h2sl_cdcg-language)

# process the headers
process_hdrs(HDRS ${HDRS} QT_HDRS ${QT_HDRS})

# process the library
process_lib(LIB ${LIB} QT_HDRS ${QT_HDRS} SRCS ${SRCS} DEPS ${DEPS})

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})
