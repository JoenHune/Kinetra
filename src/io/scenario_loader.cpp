// SPDX-License-Identifier: BSD-3-Clause
// Minimal JSON scenario loader — recursive descent parser, zero dependencies.
#include "kinetra/io/scenario_loader.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace kinetra::io {

namespace {

// ── Minimal JSON value types ─────────────────────────────────────────────────
struct JsonValue;
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;
using JsonArray  = std::vector<JsonValue>;

struct JsonValue {
    std::variant<double, std::string, bool, std::nullptr_t, JsonObject, JsonArray> data;

    [[nodiscard]] double asNumber() const { return std::get<double>(data); }
    [[nodiscard]] const std::string& asString() const { return std::get<std::string>(data); }
    [[nodiscard]] const JsonObject& asObject() const { return std::get<JsonObject>(data); }
    [[nodiscard]] const JsonArray& asArray() const { return std::get<JsonArray>(data); }

    [[nodiscard]] const JsonValue& operator[](const std::string& key) const {
        const auto& obj = asObject();
        for (const auto& [k, v] : obj) {
            if (k == key) return v;
        }
        throw std::runtime_error("JSON key not found: " + key);
    }

    [[nodiscard]] const JsonValue* find(const std::string& key) const {
        if (!std::holds_alternative<JsonObject>(data)) return nullptr;
        const auto& obj = asObject();
        for (const auto& [k, v] : obj) {
            if (k == key) return &v;
        }
        return nullptr;
    }
};

// ── Recursive descent JSON parser ────────────────────────────────────────────
class JsonParser {
public:
    explicit JsonParser(std::string_view input) : input_(input), pos_(0) {}

    JsonValue parse() {
        skipWhitespace();
        auto val = parseValue();
        skipWhitespace();
        return val;
    }

private:
    std::string_view input_;
    std::size_t pos_;

    char peek() const { return pos_ < input_.size() ? input_[pos_] : '\0'; }
    char advance() { return input_[pos_++]; }

    void skipWhitespace() {
        while (pos_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[pos_])))
            ++pos_;
    }

    void expect(char c) {
        skipWhitespace();
        if (advance() != c)
            throw std::runtime_error(std::string("Expected '") + c + "'");
    }

    JsonValue parseValue() {
        skipWhitespace();
        char c = peek();
        if (c == '"') return parseString();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') return parseNull();
        return parseNumber();
    }

    JsonValue parseString() {
        expect('"');
        std::string s;
        while (peek() != '"') {
            if (peek() == '\\') { advance(); s += advance(); }
            else s += advance();
        }
        advance();  // closing "
        return JsonValue{s};
    }

    JsonValue parseNumber() {
        skipWhitespace();
        std::size_t start = pos_;
        if (peek() == '-') advance();
        while (pos_ < input_.size() && (std::isdigit(static_cast<unsigned char>(peek()))
               || peek() == '.' || peek() == 'e' || peek() == 'E'
               || peek() == '+' || peek() == '-')) {
            if ((peek() == '+' || peek() == '-') && pos_ > start + 1
                && input_[pos_ - 1] != 'e' && input_[pos_ - 1] != 'E') break;
            advance();
        }
        double val = std::stod(std::string(input_.substr(start, pos_ - start)));
        return JsonValue{val};
    }

    JsonValue parseObject() {
        expect('{');
        JsonObject obj;
        skipWhitespace();
        if (peek() == '}') { advance(); return JsonValue{obj}; }
        while (true) {
            auto key = parseString();
            expect(':');
            auto val = parseValue();
            obj.emplace_back(std::get<std::string>(key.data), std::move(val));
            skipWhitespace();
            if (peek() == '}') { advance(); break; }
            expect(',');
        }
        return JsonValue{obj};
    }

    JsonValue parseArray() {
        expect('[');
        JsonArray arr;
        skipWhitespace();
        if (peek() == ']') { advance(); return JsonValue{arr}; }
        while (true) {
            arr.push_back(parseValue());
            skipWhitespace();
            if (peek() == ']') { advance(); break; }
            expect(',');
        }
        return JsonValue{arr};
    }

    JsonValue parseBool() {
        if (input_.substr(pos_, 4) == "true")  { pos_ += 4; return JsonValue{true}; }
        if (input_.substr(pos_, 5) == "false") { pos_ += 5; return JsonValue{false}; }
        throw std::runtime_error("Invalid bool");
    }

    JsonValue parseNull() {
        if (input_.substr(pos_, 4) == "null") { pos_ += 4; return JsonValue{nullptr}; }
        throw std::runtime_error("Invalid null");
    }
};

// ── Helper: parse a Waypoint2D from JSON ─────────────────────────────────────
Waypoint2D parseWaypoint(const JsonValue& v) {
    Waypoint2D wp;
    wp.x     = static_cast<Scalar>(v["x"].asNumber());
    wp.y     = static_cast<Scalar>(v["y"].asNumber());
    wp.theta = static_cast<Scalar>(v["theta"].asNumber());
    if (auto* t = v.find("t")) wp.t = static_cast<Scalar>(t->asNumber());
    return wp;
}

// ── Helper: parse Environment2D from JSON ────────────────────────────────────
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
                    const auto& arr = vt.asArray();
                    po.vertices.push_back(Vec2(
                        static_cast<Scalar>(arr[0].asNumber()),
                        static_cast<Scalar>(arr[1].asNumber())));
                }
                env.obstacles.push_back(po);
            }
        }
    }
    return env;
}

// ── Helper: read entire file to string ───────────────────────────────────────
std::string readFile(const std::filesystem::path& path) {
    std::ifstream ifs(path);
    if (!ifs) throw std::runtime_error("Cannot open file: " + path.string());
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
}

// ── Helper: serialize Waypoint2D to JSON ─────────────────────────────────────
void writeWaypoint(std::ostream& os, const Waypoint2D& wp) {
    os << "{\"x\":" << wp.x << ",\"y\":" << wp.y
       << ",\"theta\":" << wp.theta << ",\"t\":" << wp.t << "}";
}

// ── Helper: serialize obstacle to JSON ───────────────────────────────────────
void writeObstacle(std::ostream& os, const Obstacle& obs) {
    std::visit([&](auto&& o) {
        using T = std::decay_t<decltype(o)>;
        if constexpr (std::is_same_v<T, CircleObstacle>) {
            os << "{\"type\":\"circle\",\"center\":["
               << o.center.x() << "," << o.center.y()
               << "],\"radius\":" << o.radius << "}";
        } else if constexpr (std::is_same_v<T, RectangleObstacle>) {
            os << "{\"type\":\"rectangle\",\"min\":["
               << o.min_corner.x() << "," << o.min_corner.y()
               << "],\"max\":[" << o.max_corner.x() << "," << o.max_corner.y() << "]}";
        } else if constexpr (std::is_same_v<T, PolygonObstacle>) {
            os << "{\"type\":\"polygon\",\"vertices\":[";
            for (std::size_t i = 0; i < o.vertices.size(); ++i) {
                if (i > 0) os << ",";
                os << "[" << o.vertices[i].x() << "," << o.vertices[i].y() << "]";
            }
            os << "]}";
        }
    }, obs);
}

}  // anonymous namespace

PlanningProblem loadScenarioJSON(const std::filesystem::path& path) {
    std::string content = readFile(path);
    JsonParser parser(content);
    auto root = parser.parse();

    PlanningProblem problem;
    problem.start = parseWaypoint(root["start"]);
    problem.goal  = parseWaypoint(root["goal"]);

    if (auto* env = root.find("environment")) {
        problem.environment = parseEnvironment(*env);
    }

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

std::vector<PlanningProblem> loadScenariosFromDir(const std::filesystem::path& dir) {
    std::vector<PlanningProblem> problems;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".json") {
            problems.push_back(loadScenarioJSON(entry.path()));
        }
    }
    return problems;
}

void saveScenarioJSON(const std::filesystem::path& path, const PlanningProblem& problem) {
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("Cannot write file: " + path.string());

    ofs << "{\"start\":";
    writeWaypoint(ofs, problem.start);
    ofs << ",\"goal\":";
    writeWaypoint(ofs, problem.goal);
    ofs << ",\"environment\":{\"bounds\":{\"min\":["
        << problem.environment.bounds_min.x() << ","
        << problem.environment.bounds_min.y() << "],\"max\":["
        << problem.environment.bounds_max.x() << ","
        << problem.environment.bounds_max.y() << "]},\"obstacles\":[";

    for (std::size_t i = 0; i < problem.environment.obstacles.size(); ++i) {
        if (i > 0) ofs << ",";
        writeObstacle(ofs, problem.environment.obstacles[i]);
    }

    ofs << "]},\"options\":{\"maxIterations\":"
        << problem.options.maxIterations
        << ",\"timeLimitMs\":" << problem.options.timeLimitMs
        << ",\"goalTolerance\":" << problem.options.goalTolerance
        << "}}";
}

}  // namespace kinetra::io
