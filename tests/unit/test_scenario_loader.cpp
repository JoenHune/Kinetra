// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <filesystem>
#include "kinetra/io/scenario_loader.hpp"

using namespace kinetra;
using namespace kinetra::io;

class ScenarioLoaderTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir_;

    void SetUp() override {
        temp_dir_ = std::filesystem::temp_directory_path() / "kinetra_test_scenarios";
        std::filesystem::create_directories(temp_dir_);
    }

    void TearDown() override {
        std::filesystem::remove_all(temp_dir_);
    }
};

TEST_F(ScenarioLoaderTest, SaveAndLoad) {
    PlanningProblem original;
    original.start = {1.0, 2.0, 0.5, 0.0};
    original.goal  = {8.0, 3.0, 1.0, 0.0};
    original.environment.bounds_min = Vec2(-20, -20);
    original.environment.bounds_max = Vec2(20, 20);
    original.environment.obstacles.push_back(CircleObstacle{Vec2(3, 1), 1.5});
    original.options.maxIterations = 5000;
    original.options.timeLimitMs = 3000;

    auto path = temp_dir_ / "test.json";
    saveScenarioJSON(path, original);
    ASSERT_TRUE(std::filesystem::exists(path));

    PlanningProblem loaded = loadScenarioJSON(path);

    EXPECT_NEAR(loaded.start.x, 1.0, 1e-6);
    EXPECT_NEAR(loaded.start.y, 2.0, 1e-6);
    EXPECT_NEAR(loaded.goal.x, 8.0, 1e-6);
    EXPECT_NEAR(loaded.goal.y, 3.0, 1e-6);
    EXPECT_EQ(loaded.options.maxIterations, 5000);
}

TEST_F(ScenarioLoaderTest, LoadWithMultipleObstacles) {
    PlanningProblem original;
    original.start = {0, 0, 0, 0};
    original.goal  = {5, 5, 0, 0};
    original.environment.obstacles.push_back(CircleObstacle{Vec2(1, 1), 0.5});
    original.environment.obstacles.push_back(
        RectangleObstacle{Vec2(2, 2), Vec2(3, 3)});

    auto path = temp_dir_ / "multi_obs.json";
    saveScenarioJSON(path, original);
    PlanningProblem loaded = loadScenarioJSON(path);

    EXPECT_EQ(loaded.environment.obstacles.size(), 2u);
}

TEST_F(ScenarioLoaderTest, LoadScenariosFromDir) {
    for (int i = 0; i < 3; ++i) {
        PlanningProblem p;
        p.start = {0, 0, 0, 0};
        p.goal  = {static_cast<Scalar>(i + 1), 0, 0, 0};
        saveScenarioJSON(temp_dir_ / ("scenario_" + std::to_string(i) + ".json"), p);
    }

    auto problems = loadScenariosFromDir(temp_dir_);
    EXPECT_EQ(problems.size(), 3u);
}

TEST_F(ScenarioLoaderTest, LoadNonExistentFileThrows) {
    EXPECT_THROW(loadScenarioJSON(temp_dir_ / "nonexistent.json"),
                 std::runtime_error);
}
