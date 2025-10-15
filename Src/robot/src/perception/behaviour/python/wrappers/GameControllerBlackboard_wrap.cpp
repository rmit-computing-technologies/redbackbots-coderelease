class_<GameControllerBlackboard>("GameControllerBlackboard").
   def_readonly("connect"         , &GameControllerBlackboard::connect         ).
   def_readonly("connected"       , &GameControllerBlackboard::connected       ).
   def_readonly("active"          , &GameControllerBlackboard::active          ).
   def_readonly("data"            , &GameControllerBlackboard::data            ).
   def_readonly("gameState"       , &GameControllerBlackboard::gameState       ).
   def_readonly("our_team"        , &GameControllerBlackboard::our_team        ).
   def_readonly("opponent_team"   , &GameControllerBlackboard::opponent_team   ).
   def_readonly("left_team"       , &GameControllerBlackboard::leftTeam        ).
   def_readonly("player_number"   , &GameControllerBlackboard::player_number   );
