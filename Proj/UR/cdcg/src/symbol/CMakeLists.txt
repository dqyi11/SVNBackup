# GENGETOPT FILES
set(GGOS 
    func_kernel_demo.ggo)


# HEADER FILES
set(HDRS
    h2sl_cdcg/func_kernel.h)

# SOURCE FILES
set(SRCS
    func_kernel.cc)

# BINARY SOURCE FILES 
set(BIN_SRCS
    func_kernel_demo.cc)

# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES} ${H2SL_LIBRARIES})

# LIBRARY NAME
set(LIB h2sl_cdcg-symbol)

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})

