class_<SharedStateEstimationBundle>("SharedStateEstimationBundle")
    .def_readonly("haveBallUpdate", &SharedStateEstimationBundle::haveBallUpdate)
    .def_readonly("haveTeamBallUpdate", &SharedStateEstimationBundle::haveTeamBallUpdate)
    .def_readonly("robotPos", &SharedStateEstimationBundle::robotPos);
