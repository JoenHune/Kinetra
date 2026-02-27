// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <sstream>
#include "kinetra/io/json_export.hpp"

using namespace kinetra;
using namespace kinetra::io;

TEST(JSONExport, TrajectoryJSON) {
    Trajectory2D traj;
    traj.append({1.0, 2.0, 0.5, 0.0});
    traj.append({3.0, 4.0, 1.0, 0.1});

    std::ostringstream os;
    toJSON(os, traj, "path");
    std::string json = os.str();

    EXPECT_NE(json.find("\"path\""), std::string::npos);
    EXPECT_NE(json.find("\"x\":1"), std::string::npos);
    EXPECT_NE(json.find("\"x\":3"), std::string::npos);
}

TEST(JSONExport, PlanningResultJSON) {
    PlanningResult result;
    result.status = SolveStatus::kSuccess;
    result.plannerName = "TestPlanner";
    result.solveTimeMs = 12.5;
    result.cost = 3.14;
    result.trajectory.append({0, 0, 0, 0});
    result.trajectory.append({1, 0, 0, 0.1});

    std::ostringstream os;
    toJSON(os, result);
    std::string json = os.str();

    EXPECT_NE(json.find("\"status\":\"Success\""), std::string::npos);
    EXPECT_NE(json.find("\"planner\":\"TestPlanner\""), std::string::npos);
}

TEST(JSONExport, EnvironmentJSON) {
    Environment2D env;
    env.bounds_min = Vec2(-5, -5);
    env.bounds_max = Vec2(5, 5);
    env.obstacles.push_back(CircleObstacle{Vec2(0, 0), 1.0});
    env.obstacles.push_back(RectangleObstacle{Vec2(-2, -2), Vec2(-1, -1)});

    std::ostringstream os;
    toJSON(os, env);
    std::string json = os.str();

    EXPECT_NE(json.find("\"type\":\"circle\""), std::string::npos);
    EXPECT_NE(json.find("\"type\":\"rectangle\""), std::string::npos);
    EXPECT_NE(json.find("\"bounds\""), std::string::npos);
}

TEST(JSONExport, PolygonObstacleJSON) {
    Environment2D env;
    env.bounds_min = Vec2(-5, -5);
    env.bounds_max = Vec2(5, 5);
    PolygonObstacle po;
    po.vertices = {Vec2(0, 0), Vec2(1, 0), Vec2(0.5, 1)};
    env.obstacles.push_back(po);

    std::ostringstream os;
    toJSON(os, env);
    std::string json = os.str();

    EXPECT_NE(json.find("\"type\":\"polygon\""), std::string::npos);
    EXPECT_NE(json.find("\"vertices\""), std::string::npos);
}

TEST(JSONExport, VisualizationHTML) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};

    PlanningResult result;
    result.status = SolveStatus::kSuccess;
    result.plannerName = "Test";
    result.trajectory.append({0, 0, 0, 0});
    result.trajectory.append({5, 0, 0, 1});

    std::string html = generateVisualizationHTML(problem, result);

    EXPECT_NE(html.find("<!DOCTYPE html>"), std::string::npos);
    EXPECT_NE(html.find("Plotly.newPlot"), std::string::npos);
}

TEST(JSONExport, BenchmarkToJSON) {
    PlanningResult r1, r2;
    r1.plannerName = "A"; r1.status = SolveStatus::kSuccess;
    r2.plannerName = "B"; r2.status = SolveStatus::kTimeout;

    std::ostringstream os;
    benchmarkToJSON(os, {r1, r2});
    std::string json = os.str();

    EXPECT_EQ(json.front(), '[');
    EXPECT_EQ(json.back(), ']');
    EXPECT_NE(json.find("\"planner\":\"A\""), std::string::npos);
    EXPECT_NE(json.find("\"planner\":\"B\""), std::string::npos);
}
