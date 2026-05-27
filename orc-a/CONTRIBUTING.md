# Contributing to ORC

Thanks for your interest in contributing to ORC (Open Robotics Control)! This
document describes how to build the project, run the tests, and submit changes.

By participating in this project, you agree to abide by our
[Code of Conduct](CODE_OF_CONDUCT.md).

## Getting the source

```bash
git clone --recurse-submodules <repo-url> orc
cd orc
```

If you cloned without `--recurse-submodules`, run `git submodule update --init
--recursive` once inside the repo.

## Building

### C++ (CMake)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Common options are defined in the top-level `CMakeLists.txt`. A debug build
(`-DCMAKE_BUILD_TYPE=Debug`) is recommended while developing.

### Python bindings

The Python package lives under `python/` and is built via scikit-build-core:

```bash
pip install .
```

For an editable development install:

```bash
pip install -e . --no-build-isolation
```

## Running the tests

### C++ (ctest)

After building:

```bash
ctest --test-dir build --output-on-failure
```

### Python (pytest)

```bash
pytest tests/python
```

Please add or update tests whenever you change behavior. New features without
tests are unlikely to be merged.

## Code style and linting

The repo uses automated formatting and linting. The canonical configuration
lives in:

- `.clang-format` — C++ formatting
- `.clang-tidy` — C++ static analysis
- `cmake-format.yaml` — CMake formatting
- `.editorconfig` — whitespace/indentation defaults
- `pyproject.toml` — Python tool configuration
- `.pre-commit-config.yaml` — orchestrates the above

We strongly recommend installing [pre-commit](https://pre-commit.com) and
enabling the hooks locally:

```bash
pip install pre-commit
pre-commit install
```

Then `pre-commit run --all-files` will apply the same checks that run in CI.
See `.pre-commit-config.yaml` for the exact tool set and versions in use.

## Pull request workflow

1. Fork the repo (or create a feature branch if you have write access) and
   branch from `main`:

   ```bash
   git checkout -b feat/my-change main
   ```

2. Make your changes with focused, logically-grouped commits.
3. Use [Conventional Commits](https://www.conventionalcommits.org/) for commit
   messages, e.g.:
   - `feat: add Cartesian impedance controller`
   - `fix: handle empty trajectory in deserializer`
   - `refactor: simplify filter template parameters`
   - `docs: clarify build instructions`
   - `test: add edge cases for SplineJointInterpolator`
4. Include tests for new behavior and make sure `ctest` and `pytest` pass
   locally.
5. Run `pre-commit run --all-files` to catch style issues before pushing.
6. Open a pull request against `main`. Fill out the PR template, describe the
   motivation, and link any related issues.
7. A maintainer will review. Be prepared to iterate on feedback.

## Reporting bugs and requesting features

Please use the issue templates:

- [Bug report](.github/ISSUE_TEMPLATE/bug_report.yml)
- [Feature request](.github/ISSUE_TEMPLATE/feature_request.yml)

For security vulnerabilities, do **not** open a public issue — see
[SECURITY.md](SECURITY.md) for the private disclosure process.

## Questions

For general questions and discussion, please use GitHub Discussions rather than
the issue tracker.
