#!/usr/bin/env bash
set -euo pipefail

# Best-effort: mark workspace safe for git. Skipped silently when .gitconfig is
# bind-mounted readonly from host (the writable-target check below avoids exit 4).
if [ -w "$HOME/.gitconfig" ] || [ ! -e "$HOME/.gitconfig" ]; then
    git config --global --add safe.directory /workspaces/orc-a
fi

# Deps already in the image; --no-build-isolation skips re-downloading them.
pip install -e . --no-build-isolation

# Build Sphinx + Doxygen documentation. Doxygen XML lands in
# build/doc/doxygen/xml, which is the path Breathe in doc/conf.py reads.
# Mirrors .github/workflows/docs.yml so devcontainer output matches CI.
cmake -S . -B build -GNinja -DBUILD_DOXYGEN=ON
cmake --build build --target docs
sphinx-build -b html doc doc/_build/html

cat <<'EOF'

========================================================================
  orc-a devcontainer ready.

  Python example (MuJoCo viewer):
      python examples/python/simulate_kinova.py

  Other Python examples:
      ls examples/python/

  C++ build (library + tests + examples):
      cmake -S . -B build -GNinja -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
      cmake --build build

  Run C++ tests:
      ctest --test-dir build --output-on-failure

  Rebuild documentation:
      cmake --build build --target docs
      sphinx-build -b html doc doc/_build/html
      # Open doc/_build/html/index.html

  GUI note: on Linux hosts, run `xhost +local:docker` once on the host
  so the MuJoCo viewer can reach the X server.
========================================================================
EOF
