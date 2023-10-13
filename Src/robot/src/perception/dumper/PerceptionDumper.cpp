#include "perception/dumper/PerceptionDumper.hpp"

#include "blackboard/Blackboard.hpp"

// Full blackboard modules are required
#include "blackboard/modulesList.hpp"

PerceptionDumper::PerceptionDumper(const char *path)
   : path(path), of(path)
{
}

PerceptionDumper::~PerceptionDumper()
{
}

const std::string &PerceptionDumper::getPath() const
{
   return path;
}

void PerceptionDumper::dump(Blackboard *blackboard)
{
   blackboard->serialise(of);
}
