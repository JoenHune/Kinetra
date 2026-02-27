// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Emscripten Embind bindings — exposes the Kinetra C++ planning library
// to JavaScript in the browser via WebAssembly.
//
// Each entry point:
//   1. Receives a JSON string describing the planning problem
//   2. Parses it into C++ types (PlanningProblem + algorithm options)
//   3. Runs the actual C++ algorithm
//   4. Serialises the result to a JSON string and returns it
//
// This is the ONLY file that needs Emscripten headers; the rest of the
// library remains toolchain-agnostic.

#include <emscripten/bind.h>

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

// Kinetra headers
#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/trajectory.hpp"
#include "kinetra/core/types.hpp"
#include "kinetra/io/json_export.hpp"
#include "kinetra/planners/ilqr.hpp"
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/planners/stomp.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/omnidirectional.hpp"
#include "kinetra/spaces/se2.hpp"

using namespace kinetra;

// ═════════════════════════════════════════════════════════════════════════════
// Minimal recursive-descent JSON parser (zero-dep, identical logic to
// scenario_loader.cpp but kept self-contained for the Wasm build which
// deliberately excludes std::filesystem).
// ═════════════════════════════════════════════════════════════════════════════
namespace {

struct JsonValue;
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;
using JsonArray  = std::vector<JsonValue>;

struct JsonValue {
    std::variant<double, std::string, bool, std::nullptr_t,
                 JsonObject, JsonArray> data;

    double asNumber() const { return std::get<double>(data); }
    const std::string& asString() const { return std::get<std::string>(data); }
    const JsonObject& asObject() const { return std::get<JsonObject>(data); }
    const JsonArray& asArray() const { return std::get<JsonArray>(data); }

    const JsonValue& operator[](const std::string& key) const {
        for (const auto& [k, v] : asObject())
            if (k == key) return v;
        throw std::runtime_error("JSON key not found: " + key);
    }
    const JsonValue* find(const std::string& key) const {
        if (!std::holds_alternative<JsonObject>(data)) return nullptr;
        for (const auto& [k, v] : asObject())
            if (k == key) return &v;
        return nullptr;
    }
};

class JsonParser {
public:
    explicit JsonParser(std::string_view in) : s_(in) {}
    JsonValue parse() { ws(); auto v = value(); ws(); return v; }
private:
    std::string_view s_;
    std::size_t p_{0};
    char peek() const { return p_ < s_.size() ? s_[p_] : '\0'; }
    char next() { return s_[p_++]; }
    void ws() { while (p_ < s_.size() && std::isspace((unsigned char)s_[p_])) ++p_; }
    void expect(char c) { ws(); if (next() != c) throw std::runtime_error(std::string("Expected '") + c + "'"); }

    JsonValue value() {
        ws();
        char c = peek();
        if (c == '"') return str();
        if (c == '{') return obj();
        if (c == '[') return arr();
        if (c == 't' || c == 'f') return boolVal();
        if (c == 'n') { p_ += 4; return JsonValue{nullptr}; }
        return num();
    }
    JsonValue str() {
        expect('"');
        std::string r;
        while (peek() != '"') { if (peek() == '\\') { next(); r += next(); } else r += next(); }
        next();
        return JsonValue{r};
    }
    JsonValue num() {
        ws();
        auto st = p_;
        if (peek() == '-') next();
        while (p_ < s_.size() && (std::isdigit((unsigned char)peek()) || peek() == '.' ||
               peek() == 'e' || peek() == 'E' || peek() == '+' || peek() == '-')) {
            if ((peek() == '+' || peek() == '-') && p_ > st + 1 &&
                s_[p_-1] != 'e' && s_[p_-1] != 'E') break;
            next();
        }
        return JsonValue{std::stod(std::string(s_.substr(st, p_ - st)))};
    }
    JsonValue obj() {
        expect('{'); JsonObject o; ws();
        if (peek() == '}') { next(); return JsonValue{o}; }
        for (;;) {
            auto k = str(); expect(':'); auto v = value();
            o.emplace_back(std::get<std::string>(k.data), std::move(v));
            ws(); if (peek() == '}') { next(); break; } expect(',');
        }
        return JsonValue{o};
    }
    JsonValue arr() {
        expect('['); JsonArray a; ws();
        if (peek() == ']') { next(); return JsonValue{a}; }
        for (;;) {
            a.push_back(value()); ws();
            if (peek() == ']') { next(); break; } expect(',');
        }
        return JsonValue{a};
    }
    JsonValue boolVal() {
        if (s_.substr(p_, 4) == "true")  { p_ += 4; return JsonValue{true}; }
        if (s_.substr(p_, 5) == "false") { p_ += 5; return JsonValue{false}; }
        throw std::runtime_error("Invalid bool");
    }
};

// ── Parse helpers ───────────────────────────────────────────────────────────

Waypoint2D parseWaypoint(const JsonValue& v) {
    Waypoint2D wp;
    wp.x     = static_cast<Scalar>(v["x"].asNumber());
    wp.y     = static_cast<Scalar>(v["y"].asNumber());
    if (auto* th = v.find("theta")) wp.theta = static_cast<Scalar>(th->asNumber());
    if (auto* t  = v.find("t"))     wp.t     = static_cast<Scalar>(t->asNumber());
    return wp;
}

Environment2D parseEnvironment(const JsonValue& v) {
    Environment2D env;
    if (auto* bounds = v.find("bounds")) {
        const auto& bmin = (*bounds)["min"].asArray();
        const auto& bmax = (*bounds)["max"].asArray();
        env.bounds_min = Vec2(static_cast<Scalar>(bmin[0].asNumber()),
                              static_cast<Scalar>(bmin[1].asNumber()));
        env.bounds_max = Vec2(static_cast<Scalar>(bmax[0].asNumber()),
                              static_cast<Scalar>(bmax[1].asNumber()));
    }
    if (auto* obs = v.find("obstacles")) {
        for (const auto& o : obs->asArray()) {
            auto type = o["type"].asString();
            if (type == "circle") {
                const auto& c = o["center"].asArray();
                CircleObstacle co;
                co.center = Vec2(static_cast<Scalar>(c[0].asNumber()),
                                 static_cast<Scalar>(c[1].asNumber()));
                co.radius = static_cast<Scalar>(o["radius"].asNumber());
                env.obstacles.push_back(co);
            } else if (type == "rectangle") {
                const auto& mn = o["min"].asArray();
                const auto& mx = o["max"].asArray();
                RectangleObstacle ro;
                ro.min_corner = Vec2(static_cast<Scalar>(mn[0].asNumber()),
                                     static_cast<Scalar>(mn[1].asNumber()));
                ro.max_corner = Vec2(static_cast<Scalar>(mx[0].asNumber()),
                                     static_cast<Scalar>(mx[1].asNumber()));
                env.obstacles.push_back(ro);
            } else if (type == "polygon") {
                PolygonObstacle po;
                for (const auto& vt : o["vertices"].asArray()) {
                    const auto& a = vt.asArray();
                    po.vertices.push_back(Vec2(static_cast<Scalar>(a[0].asNumber()),
                                               static_cast<Scalar>(a[1].asNumber())));
                }
                env.obstacles.push_back(po);
            }
        }
    }
    return env;
}

PlanningProblem parseProblem(const JsonValue& root) {
    PlanningProblem problem;
    problem.start = parseWaypoint(root["start"]);
    problem.goal  = parseWaypoint(root["goal"]);
    if (auto* env = root.find("environment"))
        problem.environment = parseEnvironment(*env);
    if (auto* opts = root.find("options")) {
        if (auto* mi = opts->find("maxIterations"))
            problem.options.maxIterations = static_cast<int>(mi->asNumber());
        if (auto* tl = opts->find("timeLimitMs"))
            problem.options.timeLimitMs = tl->asNumber();
        if (auto* gt = opts->find("goalTolerance"))
            problem.options.goalTolerance = static_cast<Scalar>(gt->asNumber());
    }
    return problem;
}

// ── Build OccupancyGrid2D from an Environment2D ─────────────────────────────

collision::OccupancyGrid2D buildGrid(const Environment2D& env,
                                     Scalar resolution = 0.1) {
    collision::OccupancyGrid2D grid(
        env.bounds_min.x(), env.bounds_min.y(),
        env.bounds_max.x(), env.bounds_max.y(),
        resolution);

    for (const auto& obs : env.obstacles) {
        std::visit([&](auto&& o) {
            using T = std::decay_t<decltype(o)>;
            if constexpr (std::is_same_v<T, CircleObstacle>)
                grid.addCircleObstacle(o.center, o.radius);
            else if constexpr (std::is_same_v<T, RectangleObstacle>)
                grid.addRectObstacle(o.min_corner, o.max_corner);
            else if constexpr (std::is_same_v<T, PolygonObstacle>)
                grid.addPolygonObstacle(o.vertices);
        }, obs);
    }
    return grid;
}

// ── Serialize PlanningResult to JSON string ─────────────────────────────────

std::string resultToJson(const PlanningProblem& problem,
                         const PlanningResult& result) {
    std::ostringstream os;
    io::toJSON(os, problem, result);
    return os.str();
}

// ── Generic obstacle cost function for STOMP ────────────────────────────────

planners::TrajectoryCostFn makeObstacleCost(const collision::OccupancyGrid2D& grid) {
    return [&grid](const MatX& traj) -> VecX {
        int N = static_cast<int>(traj.rows());
        VecX costs = VecX::Zero(N);
        for (int i = 0; i < N; ++i) {
            Vec2 p(traj(i, 0), traj(i, 1));
            Scalar sd = grid.signedDistance(p);
            if (sd < 0.5) {
                // Quadratic penalty for being close or inside obstacles
                costs[i] = (0.5 - sd) * (0.5 - sd) * 100.0;
            }
        }
        return costs;
    };
}

}  // anonymous namespace

// ═════════════════════════════════════════════════════════════════════════════
// Public API (exposed to JavaScript via Embind)
// Each function: JSON string in → JSON string out
// ═════════════════════════════════════════════════════════════════════════════

/// Plan a path using RRT*.
std::string planRRTStar(const std::string& jsonInput) {
    try {
        JsonParser parser(jsonInput);
        auto root = parser.parse();
        auto problem = parseProblem(root);

        // Parse planner-specific options
        planners::RRTStarOptions opts;
        if (auto* po = root.find("plannerOptions")) {
            if (auto* v = po->find("maxIterations"))
                opts.maxIterations = static_cast<int>(v->asNumber());
            if (auto* v = po->find("stepSize"))
                opts.stepSize = static_cast<Scalar>(v->asNumber());
            if (auto* v = po->find("goalBias"))
                opts.goalBias = static_cast<Scalar>(v->asNumber());
            if (auto* v = po->find("rewireRadius"))
                opts.rewireRadius = static_cast<Scalar>(v->asNumber());
        }
        opts.timeLimitMs = problem.options.timeLimitMs;

        auto grid = buildGrid(problem.environment);

        planners::RRTStar planner(opts);
        planner.setCollisionChecker(
            [&grid](const Vec2& p) { return grid.isFree(p); },
            [&grid](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
        );

        auto result = planner.solve(problem);
        return resultToJson(problem, result);
    } catch (const std::exception& e) {
        return std::string("{\"error\":\"") + e.what() + "\"}";
    }
}

/// Optimise a trajectory using STOMP.
std::string optimizeSTOMP(const std::string& jsonInput) {
    try {
        JsonParser parser(jsonInput);
        auto root = parser.parse();
        auto problem = parseProblem(root);

        planners::STOMPOptions opts;
        if (auto* po = root.find("plannerOptions")) {
            if (auto* v = po->find("numTimesteps"))
                opts.numTimesteps = static_cast<int>(v->asNumber());
            if (auto* v = po->find("numRollouts"))
                opts.numRollouts = static_cast<int>(v->asNumber());
            if (auto* v = po->find("maxIterations"))
                opts.maxIterations = static_cast<int>(v->asNumber());
            if (auto* v = po->find("dt"))
                opts.dt = static_cast<Scalar>(v->asNumber());
            if (auto* v = po->find("noiseSigma"))
                opts.noiseSigma = static_cast<Scalar>(v->asNumber());
        }
        opts.timeLimitMs = problem.options.timeLimitMs;

        auto grid = buildGrid(problem.environment);

        planners::STOMP planner(opts);
        planner.setCostFunction(makeObstacleCost(grid));

        auto result = planner.solve(problem);
        return resultToJson(problem, result);
    } catch (const std::exception& e) {
        return std::string("{\"error\":\"") + e.what() + "\"}";
    }
}

/// Optimise a trajectory using iLQR with a specified robot model.
/// Model names: "DiffDriveSimple", "DiffDriveAccel", "AckermannSimple", "OmniSimple"
std::string optimizeiLQR(const std::string& jsonInput) {
    try {
        JsonParser parser(jsonInput);
        auto root = parser.parse();
        auto problem = parseProblem(root);

        // Read model selection & iLQR options
        std::string modelName = "DiffDriveSimple";
        if (auto* mn = root.find("model"))
            modelName = mn->asString();

        planners::iLQROptions opts;
        if (auto* po = root.find("plannerOptions")) {
            if (auto* v = po->find("maxIterations"))
                opts.maxIterations = static_cast<int>(v->asNumber());
            if (auto* v = po->find("horizon"))
                opts.horizon = static_cast<int>(v->asNumber());
            if (auto* v = po->find("dt"))
                opts.dt = static_cast<Scalar>(v->asNumber());
            if (auto* v = po->find("tolerance"))
                opts.tolerance = static_cast<Scalar>(v->asNumber());
        }
        opts.timeLimitMs = problem.options.timeLimitMs;

        PlanningResult result;

        if (modelName == "DiffDriveSimple") {
            DiffDriveSimple model;
            planners::iLQR<DiffDriveSimple> planner(model, opts);
            result = planner.solve(problem);
        } else if (modelName == "DiffDriveAccel") {
            DiffDriveAccel model;
            planners::iLQR<DiffDriveAccel> planner(model, opts);
            result = planner.solve(problem);
        } else if (modelName == "AckermannSimple") {
            AckermannSimple model;
            planners::iLQR<AckermannSimple> planner(model, opts);
            result = planner.solve(problem);
        } else if (modelName == "OmniSimple") {
            OmniSimple model;
            planners::iLQR<OmniSimple> planner(model, opts);
            result = planner.solve(problem);
        } else {
            return "{\"error\":\"Unknown model: " + modelName + "\"}";
        }

        return resultToJson(problem, result);
    } catch (const std::exception& e) {
        return std::string("{\"error\":\"") + e.what() + "\"}";
    }
}

/// Run a two-stage pipeline: RRT* for initial path → iLQR/STOMP for
/// trajectory optimisation. Returns both the initial path and the
/// optimised trajectory.
std::string planPipeline(const std::string& jsonInput) {
    try {
        JsonParser parser(jsonInput);
        auto root = parser.parse();
        auto problem = parseProblem(root);

        std::string optimizer = "iLQR";
        if (auto* v = root.find("optimizer"))
            optimizer = v->asString();

        std::string modelName = "DiffDriveSimple";
        if (auto* mn = root.find("model"))
            modelName = mn->asString();

        auto grid = buildGrid(problem.environment);

        // ── Stage 1: RRT* path planning ─────────────────────────────────
        planners::RRTStarOptions rrtOpts;
        rrtOpts.maxIterations = 3000;
        rrtOpts.stepSize = 0.5;
        rrtOpts.goalBias = 0.1;
        rrtOpts.timeLimitMs = problem.options.timeLimitMs / 2.0;

        if (auto* po = root.find("rrtOptions")) {
            if (auto* v = po->find("maxIterations"))
                rrtOpts.maxIterations = static_cast<int>(v->asNumber());
            if (auto* v = po->find("stepSize"))
                rrtOpts.stepSize = static_cast<Scalar>(v->asNumber());
            if (auto* v = po->find("goalBias"))
                rrtOpts.goalBias = static_cast<Scalar>(v->asNumber());
        }

        planners::RRTStar rrt(rrtOpts);
        rrt.setCollisionChecker(
            [&grid](const Vec2& p) { return grid.isFree(p); },
            [&grid](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
        );

        auto rrtResult = rrt.solve(problem);

        // ── Stage 2: Trajectory optimisation ────────────────────────────
        PlanningResult optResult;

        if (rrtResult.success()) {
            // Use RRT* path as initial guess
            problem.initialGuess = rrtResult.trajectory;

            if (optimizer == "STOMP") {
                planners::STOMPOptions stompOpts;
                stompOpts.numTimesteps = std::max(20, static_cast<int>(rrtResult.trajectory.size()));
                stompOpts.timeLimitMs = problem.options.timeLimitMs / 2.0;
                if (auto* po = root.find("stompOptions")) {
                    if (auto* v = po->find("numRollouts"))
                        stompOpts.numRollouts = static_cast<int>(v->asNumber());
                    if (auto* v = po->find("noiseSigma"))
                        stompOpts.noiseSigma = static_cast<Scalar>(v->asNumber());
                }

                planners::STOMP stomp(stompOpts);
                stomp.setCostFunction(makeObstacleCost(grid));
                optResult = stomp.solve(problem);
            } else {
                // Default: iLQR
                planners::iLQROptions ilqrOpts;
                ilqrOpts.timeLimitMs = problem.options.timeLimitMs / 2.0;
                if (auto* po = root.find("ilqrOptions")) {
                    if (auto* v = po->find("horizon"))
                        ilqrOpts.horizon = static_cast<int>(v->asNumber());
                    if (auto* v = po->find("dt"))
                        ilqrOpts.dt = static_cast<Scalar>(v->asNumber());
                }

                if (modelName == "DiffDriveSimple") {
                    DiffDriveSimple m;
                    planners::iLQR<DiffDriveSimple> planner(m, ilqrOpts);
                    optResult = planner.solve(problem);
                } else if (modelName == "DiffDriveAccel") {
                    DiffDriveAccel m;
                    planners::iLQR<DiffDriveAccel> planner(m, ilqrOpts);
                    optResult = planner.solve(problem);
                } else if (modelName == "AckermannSimple") {
                    AckermannSimple m;
                    planners::iLQR<AckermannSimple> planner(m, ilqrOpts);
                    optResult = planner.solve(problem);
                } else {
                    OmniSimple m;
                    planners::iLQR<OmniSimple> planner(m, ilqrOpts);
                    optResult = planner.solve(problem);
                }
            }
        }

        // ── Build combined JSON result ──────────────────────────────────
        std::ostringstream os;
        os << "{\"rrt\":";
        io::toJSON(os, rrtResult);
        os << ",\"optimized\":";
        io::toJSON(os, optResult);
        os << ",\"problem\":{";
        os << "\"start\":{\"x\":" << problem.start.x
           << ",\"y\":" << problem.start.y
           << ",\"theta\":" << problem.start.theta << "},";
        os << "\"goal\":{\"x\":" << problem.goal.x
           << ",\"y\":" << problem.goal.y
           << ",\"theta\":" << problem.goal.theta << "},";
        os << "\"environment\":";
        io::toJSON(os, problem.environment);
        os << "}}";

        return os.str();
    } catch (const std::exception& e) {
        return std::string("{\"error\":\"") + e.what() + "\"}";
    }
}

/// Return library version info.
std::string getVersion() {
    return "{\"name\":\"Kinetra\",\"version\":\"0.4.0\","
           "\"algorithms\":[\"RRT*\",\"STOMP\",\"iLQR\"],"
           "\"models\":[\"DiffDriveSimple\",\"DiffDriveAccel\","
           "\"AckermannSimple\",\"OmniSimple\"]}";
}

// ═════════════════════════════════════════════════════════════════════════════
// Embind registration
// ═════════════════════════════════════════════════════════════════════════════
EMSCRIPTEN_BINDINGS(kinetra) {
    emscripten::function("planRRTStar",     &planRRTStar);
    emscripten::function("optimizeSTOMP",   &optimizeSTOMP);
    emscripten::function("optimizeiLQR",    &optimizeiLQR);
    emscripten::function("planPipeline",    &planPipeline);
    emscripten::function("getVersion",      &getVersion);
}
