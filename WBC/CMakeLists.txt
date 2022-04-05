aux_source_directory(./src WBC_SRC)
aux_source_directory(../Goldfarb_Optimizer QP_SOLVER_SRC)
aux_source_directory(../FloatingBasedDynamics/src ROBOT_DYNAMICS_SRC)
aux_source_directory(../TOOLS/src TOOLS_SRC)

pybind11_add_module(WBC_CONTROLLER ${WBC_SRC} 
${QP_SOLVER_SRC} 
${ROBOT_DYNAMICS_SRC}
${TOOLS_SRC})
# add_library(wbc_lib SHARED ${WBC_SRC})

# target_link_libraries(wbc_lib FloatingBasedDynamics Goldfarb_Optimizer)

