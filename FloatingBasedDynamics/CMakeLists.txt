aux_source_directory(./src FLOATING_BASED_DYNAMICS_SRC)



add_library(FloatingBasedDynamics SHARED ${FLOATING_BASED_DYNAMICS_SRC})

target_link_libraries(FloatingBasedDynamics Tools)

