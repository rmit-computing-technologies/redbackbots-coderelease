class_<BehaviourSharedData>("BehaviourSharedData")
    .def_readwrite("secondsSinceLastKick", &BehaviourSharedData::secondsSinceLastKick)
    .def_readwrite("role", &BehaviourSharedData::role)
    .def_readwrite("playingBall", &BehaviourSharedData::playingBall)
    .def_readwrite("playingBallScore", &BehaviourSharedData::playingBallScore)
    .def_readwrite("needAssistance", &BehaviourSharedData::needAssistance)
    .def_readwrite("isAssisting", &BehaviourSharedData::isAssisting)
    .def_readwrite("isKickedOff", &BehaviourSharedData::isKickedOff)
    .def_readwrite("walkingToX", &BehaviourSharedData::walkingToX)
    .def_readwrite("walkingToY", &BehaviourSharedData::walkingToY)
    .def_readwrite("walkingToH", &BehaviourSharedData::walkingToH)
    .def_readwrite("kickNotification", &BehaviourSharedData::kickNotification);
