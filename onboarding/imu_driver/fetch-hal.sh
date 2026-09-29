#!/usr/bin/env bash
# Reconstruct Drivers/ from the upstream sources pinned in hal.lock.
#
#   ./fetch-hal.sh          fetch if Drivers/ is missing
#   ./fetch-hal.sh --force  fetch even if it already exists
#
# Drivers/ is generated vendor code and is not tracked in git. Each entry in
# hal.lock names a repository and an exact commit; git object ids are content
# hashes, so a fetch either produces those exact bytes or fails.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK="$PROJECT_DIR/hal.lock"
DEST="$PROJECT_DIR/Drivers"

if [ -d "$DEST" ] && [ "${1:-}" != "--force" ]; then
  echo "Drivers/ already present, nothing to do (use --force to refetch)"
  exit 0
fi

rm -rf "$DEST"
mkdir -p "$DEST"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

while IFS=$'\t' read -r repo commit src dst; do
  case "$repo" in ''|'#'*) continue ;; esac

  echo "fetching $repo @ ${commit:0:12} -> Drivers/$dst"
  checkout="$WORK/$(echo "$repo" | tr '/' '_')"
  mkdir -p "$checkout"

  git -C "$checkout" init -q
  git -C "$checkout" remote add origin "https://github.com/$repo.git"
  git -C "$checkout" config extensions.partialClone origin

  # Only a subdirectory is wanted from the big Cube repositories; the split
  # HAL repos are taken whole.
  if [ "$src" != "." ]; then
    git -C "$checkout" sparse-checkout set --no-cone "$src"
  fi

  git -C "$checkout" fetch -q --depth 1 --filter=blob:none origin "$commit"
  git -C "$checkout" checkout -q FETCH_HEAD

  if [ ! -d "$checkout/$src" ]; then
    echo "error: $repo@$commit has no $src" >&2
    exit 1
  fi

  mkdir -p "$(dirname "$DEST/$dst")"
  cp -R "$checkout/$src" "$DEST/$dst"
  # Strip repository metadata; only the sources are wanted.
  rm -rf "$DEST/$dst/.git" "$DEST/$dst/.github"
done < "$LOCK"

echo "Drivers/ reconstructed ($(du -sh "$DEST" | cut -f1))"
