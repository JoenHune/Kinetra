# Implement Phase

Write or modify C++ code to realise the improvements from the design phase.

## Guidelines

- Follow test-driven development: write the test FIRST.
- Use C++20 features: concepts, constexpr, std::span, designated initializers.
- Keep Eigen as the only external dependency.
- Optimise for ARMv7: prefer float-friendly code when `KINETRA_USE_FLOAT` is set.
- All new public types must be documented with `///` Doxygen comments.
- Static-assert concept satisfaction for new types.
