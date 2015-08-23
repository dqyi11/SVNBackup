# GENGETOPT FILES
set(GGOS
    cllm_train.ggo)

# HEADER FILES
set(HDRS )

# SOURCE FILES
set(SRCS )

# BINARY SOURCE FILES
set(BIN_SRCS
    cllm_train.cc)

# LIBRARAY DEPENDENCIES
set(DEPS h2sl-cllm h2sl-csymbol)

# LIBRARY NAME
set(LIB )

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
#process_lib(LIB ${LIB})

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})  


