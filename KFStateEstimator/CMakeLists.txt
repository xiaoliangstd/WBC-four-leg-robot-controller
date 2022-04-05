aux_source_directory(./src KF_STATE_ESIMATOR_SRC)

# add_library(KFStateEstimator ${KF_STATE_ESIMATOR_SRC})

pybind11_add_module(KFStateEstimator ${KF_STATE_ESIMATOR_SRC})