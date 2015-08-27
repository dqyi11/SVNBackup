# GENGETOPT FILES
set(GGOS
    feature_set_cdcg_demo.ggo)


# HEADER FILES
set(HDRS
    h2sl_cdcg/feature_func_kernel.h 
    h2sl_cdcg/feature_func_kernel_object.h
    h2sl_cdcg/feature_product.h
    h2sl_cdcg/feature_set.h
    h2sl_cdcg/feature_ccv.h
    h2sl_cdcg/feature_func_kernel_matches_child.h
    )


# SOURCE FILES
set(SRCS
    feature_func_kernel.cc
    feature_func_kernel_object.cc
    feature_product.cc
    feature_set.cc
    feature_ccv.cc
    feature_func_kernel_matches_child.cc
    )

# BINARY SOURCE FILES
set(BIN_SRCS
    feature_set_cdcg_demo.cc
    )


# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES} ${H2SL_LIBRARIES} h2sl_cdcg-symbol)


# LIBRARY NAME
set(LIB h2sl_cdcg-llm) 

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})
