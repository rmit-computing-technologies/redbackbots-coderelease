class_<Blackboard>("Blackboard")
    .def_readonly("motion", &Blackboard::motion)
    .def_readonly("vision", &Blackboard::vision)
    .def_readonly("stateEstimation", &Blackboard::stateEstimation)
    .def_readonly("behaviour", &Blackboard::behaviour)
    .def_readonly("gameController", &Blackboard::gameController)
    .def_readonly("receiver", &Blackboard::receiver)
    .add_property("config", make_getter(&Blackboard::config, return_value_policy<return_by_value>()))
    .def_readonly("kinematics", &Blackboard::kinematics);

// This is VERY important to enable use of shared pointers in Blackboard with Python
// Register Shared python pointers
register_ptr_to_python< boost::shared_ptr<MotionBlackboard> >();
register_ptr_to_python< boost::shared_ptr<VisionBlackboard> >();
register_ptr_to_python< boost::shared_ptr<StateEstimationBlackboard> >();
register_ptr_to_python< boost::shared_ptr<BehaviourBlackboard> >();
register_ptr_to_python< boost::shared_ptr<GameControllerBlackboard> >();
register_ptr_to_python< boost::shared_ptr<ReceiverBlackboard> >();
register_ptr_to_python< boost::shared_ptr<KinematicsBlackboard> >();
