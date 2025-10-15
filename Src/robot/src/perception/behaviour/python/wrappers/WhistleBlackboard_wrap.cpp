class_<WhistleBlackboard>("WhistleBlackboard").
   def_readonly("whistleDetectionState", &WhistleBlackboard::whistleDetectionState).
   def_readwrite("whistleThreadCrashed", &WhistleBlackboard::whistleThreadCrashed);

   enum_<WhistleDetectionState>("WhistleDetectionState")
      .value("dontKnow", WhistleDetectionState::dontKnow)
      .value("notDetected", WhistleDetectionState::notDetected)
      .value("isDetected", WhistleDetectionState::isDetected);
