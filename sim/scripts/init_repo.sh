#!/usr/bin/env bash
# One-time: turn this folder into a git repo and make the first commit.
# Run from WSL:  bash sim/scripts/init_repo.sh
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
# Stage 0 mission file saved earlier in QGroundControl's default folder
[ -f sim/missions/stage0_baseline_mission.plan ] || \
  cp ~/Applications/baseline_mission.plan sim/missions/stage0_baseline_mission.plan 2>/dev/null || \
  echo "!! baseline_mission.plan not found in ~/Applications - copy it to sim/missions/stage0_baseline_mission.plan yourself"
if [ -z "$(git config --global user.name)" ]; then
  read -rp "Your name for git commits: " N; git config --global user.name "$N"
  read -rp "Your GitHub email: " E; git config --global user.email "$E"
fi
git config --global --add safe.directory "$REPO"
[ -d .git ] || git init -b main
git add .
git commit -m "Stages 0-1: baseline mission, damaged-building world, documentation" || true
git log --oneline | head -3
echo "Done. Next: publish with GitHub Desktop (see docs/github_setup.md)."
