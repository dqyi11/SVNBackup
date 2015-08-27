# GENGETOPT FILES
set(GGOS
   dcg_demo_cdcg.ggo
   )

# HEADER FILES
set(HDRS
    h2sl_cdcg/dcg.h 
    h2sl_cdcg/factor.h
    h2sl_cdcg/factor_set.h
    )


# SOURCE FILES
set(SRCS
    dcg.cc
    factor.cc
    factor_set.cc
    )

# BINARY SOURCE FILES
set(BIN_SRCS
    dcg_demo_cdcg.cc
    )


# LIBRARY DEPENDENCIES
set(DEPS ${LBFGS_LIBRARY} ${LIBXML2_LIBRARIES} ${H2SL_LIBRARIES} h2sl_cdcg-symbol h2sl_cdcg-llm h2sl_cdcg-language )


# LIBRARY NAME
set(LIB h2sl_cdcg-dcg) 

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
process_lib(LIB ${LIB} SRCS ${SRCS} DEPS ${DEPS}) 

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})
