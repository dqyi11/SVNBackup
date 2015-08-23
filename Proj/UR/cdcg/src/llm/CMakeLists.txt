# GENGETOPT FILES



# HEADER FILES
set(HDRS
    h2sl/feature_func_kernel_object.h 
    h2sl/feature_ccv.h)
   

# SOURCE FILES
set(SRCS
    feature_func_kernel_object.cc
    feature_ccv.cc)

# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES})

# LIBRARY NAME
set(LIB h2sl-cllm)

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 
