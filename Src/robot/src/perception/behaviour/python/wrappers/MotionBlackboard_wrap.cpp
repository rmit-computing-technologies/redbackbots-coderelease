class_<MotionBlackboard>("MotionBlackboard")
   .add_property("sensors", &MotionBlackboard::sensors) 
   .add_property("active", &MotionBlackboard::active)
   .add_property("odometry", &MotionBlackboard::odometry);
