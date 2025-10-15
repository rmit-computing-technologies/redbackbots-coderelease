/**
 * @file sdeerialise.hpp
 * 
 * Deseralise function declarations.
 * These enable things that want to leverage serialisation to find what is required.
 * 
 * Also gives forward declarations so ordering of deserialisation methods in
 * deserialise.cpp isn't so strict.
 * 
 * @author RedbackBots
 * 
 */

#pragma once

// Blackboard
#include "blackboard/Blackboard.hpp"

// Blackboard modules
#include "blackboard/modulesList.hpp"

// Generated file from Protobuf
#include "Blackboard.pb.h"

void deserialise(Blackboard &cpp, const offnao::Blackboard &pb);