# Test Phase

Build the project and run the full test suite.

## Success Criteria

- All tests pass (0 failures).
- No sanitizer warnings (when enabled).
- Build completes without warnings on `-Wall -Wextra`.

## On Failure

- Parse CTest output to identify failing tests.
- Read the test source to understand expected behaviour.
- Propose a fix in the implementation (not the test).
