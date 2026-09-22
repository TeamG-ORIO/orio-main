#!/usr/bin/env bash
# Download FC-GQCNN-4.0-SUCTION weights. Berkeley's Box links are dead (404, upstream
# issues #133/#141); this pulls from a HuggingFace mirror of their model_zoo.zip.
set -euo pipefail

DEST="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../gqcnn" && pwd)/models}"
MODEL="FC-GQCNN-4.0-SUCTION"
ZOO="https://huggingface.co/WoodenHeart0214/gqcnn/resolve/main/model_zoo.zip"
SHA256="fb8226c7212381a91cf6bc16c71df7fa33a0f18a2dbc28e1cdf933eb1c53e48d"

if [ -f "$DEST/$MODEL/model.ckpt.index" ]; then
    echo "$MODEL already present at $DEST/$MODEL"
    exit 0
fi

mkdir -p "$DEST"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# The model zip is the first entry in model_zoo.zip, so a ranged request avoids
# downloading all 1.19 GB. The entry uses a data descriptor (sizes are 0 in the local
# header), so decompress the stream and let zlib stop at its end.
echo "Fetching $MODEL ..."
curl -fsSL -r 0-3000000 "$ZOO" -o "$TMP/head.bin"

python3 - "$TMP/head.bin" "$TMP/$MODEL.zip" <<'PY'
import struct, sys, zlib
raw = open(sys.argv[1], "rb").read()
assert raw[:4] == b"PK\x03\x04", "unexpected zip header"
n, m = struct.unpack("<HH", raw[26:30])
start = 30 + n + m
open(sys.argv[2], "wb").write(zlib.decompressobj(-15).decompress(raw[start:]))
PY

echo "$SHA256  $TMP/$MODEL.zip" | sha256sum -c - >/dev/null
unzip -qo "$TMP/$MODEL.zip" -d "$DEST"
echo "Installed $MODEL to $DEST/$MODEL"
