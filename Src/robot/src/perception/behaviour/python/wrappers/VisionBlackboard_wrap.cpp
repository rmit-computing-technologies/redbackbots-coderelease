class_<VisionBlackboard>("VisionBlackboard")
   .add_property("balls"    , &VisionBlackboard::balls    )
   .add_property("timestamp", &VisionBlackboard::timestamp)
   .add_property("image_buffer", &VisionBlackboard::py_buffer);