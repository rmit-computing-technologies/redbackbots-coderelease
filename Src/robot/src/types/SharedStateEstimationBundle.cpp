#include "types/SharedStateEstimationBundle.hpp"
#include "utils/Logger.hpp"

#include <iostream>
#include <cmath>

bool SharedStateEstimationBundle::sanityCheck()
{
    // Check for robotPos nan
    for (unsigned i = 0; i < 3; ++i)
    {
        if (std::isnan(robotPos.vec(i)))
        {
            llog(INFO) << NDEBUG_LOGSYMB << "received nan for robotPos vec" << std::endl;
            return false;
        }
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            if (std::isnan(robotPos.var(i, j)))
            {
                llog(INFO) << NDEBUG_LOGSYMB << "received nan for robotPos var" << std::endl;
                return false;
            }
        }
    }
    if (std::isnan(robotPos.weight))
    {
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for robotPos weight" << std::endl;
        return false;
    }

    // Check for ballPosRRC nan
    for (unsigned i = 0; i < 3; ++i)
    {
        if (std::isnan(ballPosRRC.vec(i)))
        {
            llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballPosRRC vec" << std::endl;
            return false;
        }
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            if (std::isnan(ballPosRRC.var(i, j)))
            {
                llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballPosRRC var" << std::endl;
                return false;
            }
        }
    }
    if (std::isnan(ballPosRRC.weight))
    {
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballPosRRC weight" << std::endl;
        return false;
    }

    // Check for ballVelRRC nan
    for (unsigned i = 0; i < 3; ++i)
    {
        if (std::isnan(ballVelRRC.vec(i)))
        {
            llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballVelRRC vec" << std::endl;
            return false;
        }
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            if (std::isnan(ballVelRRC.var(i, j)))
            {
                llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballVelRRC var" << std::endl;
                return false;
            }
        }
    }
    if (std::isnan(ballVelRRC.weight))
    {
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballVelRRC weight" << std::endl;
        return false;
    }

    return true;
}