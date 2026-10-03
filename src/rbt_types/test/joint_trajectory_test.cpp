#include "rbt_types/joint_trajectory.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace rbt_types
{
namespace
{

JointPathMat makePositions()
{
  JointPathMat positions(3, 2);
  positions << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5;
  return positions;
}

TEST(JointTrajectoryTest, AcceptsStrictlyIncreasingTimes)
{
  TimeVec times(3);
  times << 0.0, 0.07, 0.19;

  const JointTrajectory trajectory(makePositions(), times);

  EXPECT_EQ(trajectory.length(), 3U);
  EXPECT_EQ(trajectory.dim(), 2U);
  EXPECT_TRUE(trajectory.timeFromStart().isApprox(times));
}

TEST(JointTrajectoryTest, RejectsMismatchedTimes)
{
  EXPECT_THROW(JointTrajectory(makePositions(), TimeVec::Zero(2)), std::invalid_argument);
}

TEST(JointTrajectoryTest, RejectsNonfiniteTimes)
{
  TimeVec times(3);
  times << 0.0, 0.1, std::numeric_limits<double>::infinity();

  EXPECT_THROW(JointTrajectory(makePositions(), times), std::invalid_argument);
}

TEST(JointTrajectoryTest, RejectsNegativeTimes)
{
  TimeVec times(3);
  times << -0.1, 0.0, 0.1;

  EXPECT_THROW(JointTrajectory(makePositions(), times), std::invalid_argument);
}

TEST(JointTrajectoryTest, RejectsEqualTimes)
{
  TimeVec times(3);
  times << 0.0, 0.1, 0.1;

  EXPECT_THROW(JointTrajectory(makePositions(), times), std::invalid_argument);
}

TEST(JointTrajectoryTest, RejectsDecreasingTimes)
{
  TimeVec times(3);
  times << 0.0, 0.2, 0.1;

  EXPECT_THROW(JointTrajectory(makePositions(), times), std::invalid_argument);
}

}  // namespace
}  // namespace rbt_types
