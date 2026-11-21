#!/usr/bin/env bash
# Builds flight2 with --sim-plant and bundles everything the bench needs
# into one folder (and a .tar.gz of it):
#
#   ctv-flight-computer   the binary
#   gains/                the LQR tables (hopper fsw_bridge/export_plant_gains)
#   plant.json            what plant is linked in (vehicle, commit, target)
#   bench.json            caps, thrusts, ESC map, PA-12 geometry (edit at the bench)
#   bench.env             EC address
#   run.sh                starts the test; runs launch.sh instead if present
#   eth_soak.sh           IDLE soak + runs against a fake EC on another machine
#   BENCH_CHECKLIST.md    the bench procedure
#   MANIFEST.txt          commits, build target, sha256 of every file
#
# usage: package_bench.sh --plant <build_plant_c folder> --gains <gains folder>
#                         [--target aarch64|x86_64] [--out <folder>]
# Run from WSL / Linux. aarch64 needs gcc-aarch64-linux-gnu and the
# aarch64-unknown-linux-gnu Rust target.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
luna="$(cd "$here/../../.." && pwd)"
plant="" gains="" target="aarch64" out=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --plant) plant="$2"; shift 2 ;;
        --gains) gains="$2"; shift 2 ;;
        --target) target="$2"; shift 2 ;;
        --out) out="$2"; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
[[ -f "$plant/plant.json" ]] || { echo "--plant must be a build_plant_c folder (no plant.json in '$plant')" >&2; exit 2; }
for t in tgrid xref k1flat k2grid unom; do
    [[ -f "$gains/$t.csv" ]] || { echo "--gains is missing $t.csv" >&2; exit 2; }
done
case "$target" in
    aarch64) triple=aarch64-unknown-linux-gnu; want=arm64
             export CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc
             export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc ;;
    x86_64)  triple=x86_64-unknown-linux-gnu; want=x86_64 ;;
    *) echo "--target must be aarch64 or x86_64" >&2; exit 2 ;;
esac
grep -q "\"target\": \"$want\"" "$plant/plant.json" || {
    echo "plant.json was generated for another target; run build_plant_c('Target', '$want') first" >&2; exit 2; }
out="${out:-$luna/target/bench-$target}"
target_dir="${CARGO_TARGET_DIR:-$luna/target}"

echo "== building flight2 --sim-plant for $triple"
(cd "$luna" && HOPPER_PLANT_CODE="$plant" cargo build --release -p ctv-flight-computer \
    --features sim-plant --target "$triple")

rm -rf "$out"
mkdir -p "$out/gains" "$out/logs"
cp "$target_dir/$triple/release/ctv-flight-computer" "$out/"
cp "$gains"/*.csv "$out/gains/"
[[ -f "$gains/gains.json" ]] && cp "$gains/gains.json" "$out/gains/"
cp "$plant/plant.json" "$out/"
cp "$here/bench.json" "$here/bench.env" "$here/run.sh" "$here/eth_soak.sh" "$here/BENCH_CHECKLIST.md" "$out/"
chmod +x "$out/run.sh" "$out/eth_soak.sh" "$out/ctv-flight-computer"

{
    echo "built:        $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "target:       $triple"
    commit="$(git -C "$luna" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    git -C "$luna" diff --quiet HEAD -- 2>/dev/null || commit="$commit-dirty"
    echo "luna commit:  $commit"
    echo "plant commit: $(sed -n 's/.*"git_commit": "\([^"]*\)".*/\1/p' "$plant/plant.json")"
    echo "binary:       $(file -b "$out/ctv-flight-computer")"
    echo
    (cd "$out" && find . -type f ! -name MANIFEST.txt ! -path './logs/*' | sort | xargs sha256sum)
} > "$out/MANIFEST.txt"

tar -C "$(dirname "$out")" -czf "$out.tar.gz" "$(basename "$out")"
echo "== bench package: $out  ($out.tar.gz)"
cat "$out/MANIFEST.txt" | head -6
