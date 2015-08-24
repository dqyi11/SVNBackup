# GENGETOPT FILES



# HEADER FILES
set(HDRS
    h2sl_cdcg/feature_func_kernel.h 
    h2sl_cdcg/feature_product.h
    h2sl_cdcg/feature_ccv.h)


# SOURCE FILES
set(SRCS
    feature_func_kernel.cc
    feature_product.cc
    feature_ccv.cc)

# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES})


# LIBRARY NAME
set(LIB h2sl_cdcg-llm)

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 
