#add_library( refind_object SHARED
#        main.cpp
#)

#target_link_libraries( refind_object
#    ${THIRD_PARTY_LIBS}
#)

add_executable( refind main.cpp )
target_link_libraries( refind ${THIRD_PARTY_LIBS})
