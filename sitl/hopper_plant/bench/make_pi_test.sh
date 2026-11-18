#!/usr/bin/env bash
# Builds pi_test-<target>.tar.gz: the bench package (package_bench.sh) plus a
# static fake_ec, pi_check.sh and README_PI_TEST.txt, so the flight computer
# can be checked end to end with no real EC and nothing installed.
#
# usage: make_pi_test.sh --plant <build_plant_c folder> --gains <gains folder>
#                        [--target aarch64|x86_64] [--out <parent folder>]
# Writes <out>/pi_test-<target>/ and <out>/pi_test-<target>.tar.gz
# (default out: <luna>/target/pi_test).
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
case "$target" in
    aarch64) cc=aarch64-linux-gnu-gcc ;;
    x86_64)  cc=gcc ;;
    *) echo "--target must be aarch64 or x86_64" >&2; exit 2 ;;
esac
out="${out:-$luna/target/pi_test}"
name="pi_test-$target"
dir="$out/$name"
mkdir -p "$out"

bash "$here/package_bench.sh" --plant "$plant" --gains "$gains" --target "$target" --out "$dir"
rm -f "$dir.tar.gz"   # package_bench's own tarball; this script makes the final one

echo "== fake_ec ($target, static)"
ec="$luna/firmware/fc_ec_link"
"$cc" -std=c99 -Wall -Wextra -O2 -static -o "$dir/fake_ec" \
    "$ec/fake_ec.c" "$ec/ec_side.c" "$ec/fc_ec_link.c" -lm
cp "$here/pi_check.sh" "$here/README_PI_TEST.txt" "$dir/"
chmod +x "$dir/pi_check.sh" "$dir/fake_ec"

# Checksums again, now covering the added files. Same layout as
# package_bench's: 5 header lines, a blank line, then the checksums
# (pi_check.sh checks from line 7 on).
{
    head -5 "$dir/MANIFEST.txt"
    echo
    (cd "$dir" && find . -type f ! -name MANIFEST.txt ! -path './logs/*' | sort | xargs sha256sum)
} > "$out/MANIFEST.$name.tmp"
mv "$out/MANIFEST.$name.tmp" "$dir/MANIFEST.txt"

tar -C "$out" -czf "$out/$name.tar.gz" "$name"
echo "== $out/$name.tar.gz"
head -6 "$dir/MANIFEST.txt"
