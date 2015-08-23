# GENGETOPT FILES



# HEADER FILES
set(HDRS
    h2sl/func_kernel.h)

# SOURCE FILES
set(SRCS
    func_kernel.cc)

# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES})

# LIBRARY NAME
set(LIB h2sl-csymbol)

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 
