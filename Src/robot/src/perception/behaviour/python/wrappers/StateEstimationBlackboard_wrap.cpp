class_<StateEstimationBlackboard>("StateEstimationBlackboard")
   .def_readonly("robotPos", &StateEstimationBlackboard::robotPos)
   .def_readonly("robotPosUncertainty", &StateEstimationBlackboard::robotPosUncertainty)
   .def_readonly("robotHeadingUncertainty", &StateEstimationBlackboard::robotHeadingUncertainty)
   .def_readonly("allRobotPos", &StateEstimationBlackboard::allRobotPos)
   .def_readonly("ballPosRR", &StateEstimationBlackboard::ballPosRR)
   .def_readonly("ballPosRRC", &StateEstimationBlackboard::ballPosRRC)
   .def_readonly("ballVelRRC", &StateEstimationBlackboard::ballVelRRC)
   .def_readonly("ballVel", &StateEstimationBlackboard::ballVel)
   .def_readonly("ballPos", &StateEstimationBlackboard::ballPos)
   .def_readonly("teamBallPos", &StateEstimationBlackboard::teamBallPos)
   .def_readonly("teamBallVel", &StateEstimationBlackboard::teamBallVel)
   .def_readonly("lastTeamBallUpdate", &StateEstimationBlackboard::lastTeamBallUpdate)
   .def_readonly("teamBallPosUncertainty", &StateEstimationBlackboard::teamBallPosUncertainty)
   .def_readonly("robotObstacles", &StateEstimationBlackboard::robotObstacles)
   .def_readonly("haveTeamBallUpdate", &StateEstimationBlackboard::haveTeamBallUpdate)
   .def_readwrite("ballAge", &StateEstimationBlackboard::ballAge);
