#ifndef TEAM_BALL_KF_HPP
#define TEAM_BALL_KF_HPP

#include "TeamBallKFConstants.hpp"
#include "types/AbsCoord.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class BroadcastData;
class TeamBallKFParams;
class TeamBallKFTransitioner;

class TeamBallKF
{
  public:
    explicit TeamBallKF(const EstimatorInfoInit &estimatorInfoInit);
    ~TeamBallKF();

    /* Function that gets called for every frame */
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

    TeamBallStateVector state;
    TeamBallCovarianceMatrix covariance;

  private:
    void predict(float dtInSeconds);
    bool update(const std::vector<BroadcastData> &incomingBroadcastData); // returns whether there were ballupdates from teammates

    AbsCoord getBallAbsCoord(EstimatorInfoOut &estimatorInfoOut, bool haveTeamBallUpdate);
    bool doesTeamBallNeedUpdate(const EstimatorInfoIn &estimatorInfoIn, EstimatorInfoOut &estimatorInfoOut);
    double calculateDistance(AbsCoord me, AbsCoord ball);
    int calculateRangeToBall(double distToBall);
    bool isOutsideRadius(AbsCoord myBall, AbsCoord teamBall, double radius);
    AbsCoord oldGetBallAbsCoord();
    AbsCoord oldGetBallVelAbsCoord();
    AbsCoord getBallVelAbsCoord(EstimatorInfoOut &EstimatorInfoOut, bool haveTeamBallUpdate);
    float getBallPosUncertainty();

    TeamBallKFTransitioner *transitioner;

    /* Reference to initialisation information */
    const EstimatorInfoInit &estimatorInfoInit;

    TeamBallKFParams *params;

    public:
      // This is required since the vector & matrix are static members
      // See: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // TEAM_BALL_KF_HPP
