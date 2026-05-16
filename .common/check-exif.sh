#!/usr/bin/env bash
set -euo pipefail

DIR="${1:-public}"
echo "Checking for privacy-sensitive EXIF metadata in ${DIR}/ ..."

FAILED=0

while IFS= read -r -d '' file; do
    result=$(exiftool -q \
        -if '$GPSLatitude or $GPSLongitude or $GPSPosition or $SerialNumber or $LensSerialNumber' \
        -p 'GPS=($GPSLatitude,$GPSLongitude) Serial=($SerialNumber) LensSerial=($LensSerialNumber)' \
        "$file") || true

    if [[ -n "$result" ]]; then
        echo "FAIL $file: $result"
        FAILED=1
    else
        echo "OK   $file"
    fi
done < <(find "$DIR" -type f \( -iname "*.jpg" -o -iname "*.jpeg" -o -iname "*.png" -o -iname "*.gif" -o -iname "*.webp" -o -iname "*.tiff" \) -print0 | sort -z)

echo ""
if [[ "$FAILED" -ne 0 ]]; then
    echo "ERROR: Privacy-sensitive metadata found. Run .tools/strip-exif.sh to remove it."
    exit 1
fi

echo "All files OK."
