# GENGETOPT FILES
set(GGOS
    llm_train_cdcg.ggo
    gui_demo_cdcg.ggo
   )
   

# HEADER FILES
set(HDRS )

# SOURCE FILES
set(SRCS )

# BINARY SOURCE FILES
set(BIN_SRCS
    llm_train_cdcg.cc
    gui_demo_cdcg.cc
    )

# LIBRARAY DEPENDENCIES
set(DEPS h2sl_cdcg-llm h2sl_cdcg-symbol h2sl_cdcg-dcg ${QT_LIBRARIES} ${H2SL_LIBRARIES}) 

# LIBRARY NAME
set(LIB )

# process the headers
process_hdrs(HDRS ${HDRS})

# process the library
#process_lib(LIB ${LIB})

# process the binaries
process_bin_srcs(LIB ${LIB} BIN_SRCS ${BIN_SRCS} GGOS ${GGOS} DEPS ${DEPS})  


