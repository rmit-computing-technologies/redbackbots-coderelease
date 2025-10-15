#include "types/SharedLocalisationUpdateBundle.hpp"
#include "utils/Logger.hpp"

bool SharedLocalisationUpdateBundle::sanityCheck()
{
    // Check for ballSeenFraction nan
    if (std::isnan(ballSeenFraction)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for ballSeenFraction" << std::endl;
        return false;
    }

    // Check for nans in mean
    for (int i = 0; i < SHARED_DIM; i++) {
        if (std::isnan(sharedUpdateMean(i, 0))){
            llog(INFO) << NDEBUG_LOGSYMB << "received nan in sharedUpdateMean" << std::endl;
            return false;
        }
    }

    // Check for nans in covariance
    for (int row = 0; row < SHARED_DIM; row++) {
        for (int col = 0; col < SHARED_DIM; col++) {
            if (std::isnan(sharedUpdateCovariance(row, col))){
                llog(INFO) << NDEBUG_LOGSYMB << "received nan in sharedUpdateCovariance" << std::endl;
                return false;
            }
        }
    }

    // Check for sharedDx nan
    if (std::isnan(sharedDx)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedDx" << std::endl;
        return false;
    }

    // Check for sharedDy nan
    if (std::isnan(sharedDy)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedDy" << std::endl;
        return false;
    }

    // Check for sharedDh nan
    if (std::isnan(sharedDh)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedDh" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDx nan
    if (std::isnan(sharedCovarianceDx)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedCovarianceDx" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDy nan
    if (std::isnan(sharedCovarianceDy)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedCovarianceDy" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDh nan
    if (std::isnan(sharedCovarianceDh)){
        llog(INFO) << NDEBUG_LOGSYMB << "received nan for sharedCovarianceDh" << std::endl;
        return false;
    }
    
    return true;
}
