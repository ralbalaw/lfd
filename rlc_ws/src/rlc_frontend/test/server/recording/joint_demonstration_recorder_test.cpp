#include "recording/joint_demonstration_recorder.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rlc_frontend::server_detail
{
namespace
{

using namespace std::chrono_literals;

constexpr char kJointStatesTopic[] = "/test_joint_states";

class JointDemonstrationRecorderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    node_ = std::make_shared<rclcpp::Node>("joint_demonstration_recorder_test", options);
    recorder_ =
        std::make_unique<JointDemonstrationRecorder>(node_.get(), kJointStatesTopic, 0.1);
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        kJointStatesTopic, rclcpp::SensorDataQoS());

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_->remove_node(node_);
    recorder_.reset();
    publisher_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  static sensor_msgs::msg::JointState makeJointState(std::vector<std::string> names,
                                                     std::vector<double> positions)
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = std::move(names);
    joint_state.position = std::move(positions);
    return joint_state;
  }

  void publishJointState(const sensor_msgs::msg::JointState& joint_state)
  {
    publisher_->publish(joint_state);
    for (int attempt = 0; attempt < 5; ++attempt)
    {
      executor_->spin_some(10ms);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<JointDemonstrationRecorder> recorder_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(JointDemonstrationRecorderTest, RejectsInvalidConstruction)
{
  EXPECT_THROW(JointDemonstrationRecorder(nullptr, kJointStatesTopic, 0.1),
               std::invalid_argument);
  EXPECT_THROW(JointDemonstrationRecorder(node_.get(), "", 0.1), std::invalid_argument);
  EXPECT_THROW(JointDemonstrationRecorder(node_.get(), kJointStatesTopic, -0.1),
               std::invalid_argument);
  EXPECT_THROW(JointDemonstrationRecorder(node_.get(), kJointStatesTopic,
                                          std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST_F(JointDemonstrationRecorderTest, RejectsMissingStaleAndInvalidInitialState)
{
  auto error = recorder_->startRecording();
  EXPECT_EQ(error,
            std::optional<std::string>("latest joint state is unavailable or stale"));

  auto stale_state = makeJointState({ "joint_1", "joint_2" }, { 0.0, 1.0 });
  const rclcpp::Time stale_stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  stale_state.header.stamp = stale_stamp;
  publishJointState(stale_state);

  error = recorder_->startRecording();
  EXPECT_EQ(error,
            std::optional<std::string>("latest joint state is unavailable or stale"));

  publishJointState(makeJointState({ "joint_1" }, { 0.0, 1.0 }));
  error = recorder_->startRecording();
  EXPECT_EQ(error, std::optional<std::string>(
                       "joint state name dimension does not match positions"));
}

TEST_F(JointDemonstrationRecorderTest, BuildsTrajectoryWithStrictlyIncreasingTimes)
{
  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 0.0, 0.0 }));
  EXPECT_FALSE(recorder_->startRecording());
  EXPECT_EQ(recorder_->sampleCount(), 0U);

  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 1.0, 2.0 }));
  EXPECT_FALSE(recorder_->recordSample());
  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 3.0, 4.0 }));
  EXPECT_FALSE(recorder_->recordSample());

  ASSERT_EQ(recorder_->sampleCount(), 2U);
  const auto trajectory = recorder_->buildTrajectory();
  EXPECT_EQ(trajectory.joint_trajectory.joint_names,
            (std::vector<std::string>{ "joint_1", "joint_2" }));
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 2U);
  EXPECT_EQ(trajectory.joint_trajectory.points[0].positions,
            (std::vector<double>{ 1.0, 2.0 }));
  EXPECT_EQ(trajectory.joint_trajectory.points[1].positions,
            (std::vector<double>{ 3.0, 4.0 }));

  const auto first_time =
      rclcpp::Duration(trajectory.joint_trajectory.points[0].time_from_start);
  const auto second_time =
      rclcpp::Duration(trajectory.joint_trajectory.points[1].time_from_start);
  EXPECT_EQ(first_time.nanoseconds(), 0);
  EXPECT_GT(second_time.nanoseconds(), first_time.nanoseconds());
}

TEST_F(JointDemonstrationRecorderTest, StartRecordingResetsPreviousSamples)
{
  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 0.0, 0.0 }));
  EXPECT_FALSE(recorder_->startRecording());
  EXPECT_FALSE(recorder_->recordSample());
  ASSERT_EQ(recorder_->sampleCount(), 1U);

  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 2.0, 3.0 }));
  EXPECT_FALSE(recorder_->startRecording());
  EXPECT_EQ(recorder_->sampleCount(), 0U);
  EXPECT_TRUE(recorder_->buildTrajectory().joint_trajectory.points.empty());
}

TEST_F(JointDemonstrationRecorderTest, RejectsChangedDimensionAndJointOrder)
{
  publishJointState(makeJointState({ "joint_1", "joint_2" }, { 0.0, 0.0 }));
  EXPECT_FALSE(recorder_->startRecording());
  EXPECT_FALSE(recorder_->recordSample());
  ASSERT_EQ(recorder_->sampleCount(), 1U);

  publishJointState(makeJointState({ "joint_1" }, { 1.0 }));
  auto error = recorder_->recordSample();
  EXPECT_EQ(error,
            std::optional<std::string>("joint state dimension changed while recording"));
  EXPECT_EQ(recorder_->sampleCount(), 1U);

  publishJointState(makeJointState({ "joint_2", "joint_1" }, { 2.0, 3.0 }));
  error = recorder_->recordSample();
  EXPECT_EQ(error,
            std::optional<std::string>("joint state names changed while recording"));
  EXPECT_EQ(recorder_->sampleCount(), 1U);
}

}  // namespace
}  // namespace rlc_frontend::server_detail
