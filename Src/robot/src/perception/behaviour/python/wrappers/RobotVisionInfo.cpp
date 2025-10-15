class_<RobotVisionInfo>("RobotVisionInfo")
   .def_readonly("rr"       , &RobotVisionInfo::rr      )
   .def_readonly("type"     , &RobotVisionInfo::type    );

enum_<RobotVisionInfo::Type>("RobotVisionInfoType")
   .value("rUnknown"  , RobotVisionInfo::rUnknown    )
   .value("rOwnTeam"     , RobotVisionInfo::rOwnTeam       )
   .value("rEnemyTeam"      , RobotVisionInfo::rEnemyTeam        );

