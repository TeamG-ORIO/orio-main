#!/bin/bash
# Build the single consolidated ORIO docker image (orio_docker).
# Run from the repo root (build context must be the workspace root so the
# Dockerfile's `COPY src/devel_packages` resolves, and submodules must be
# initialised: `git submodule update --init --recursive`).
set -e
REPO_ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
cd "$REPO_ROOT"
docker build -t orio_docker -f src/devel_packages/orio_bringup/docker/Dockerfile .
