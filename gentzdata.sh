#!/bin/bash

set -e

BASEDIR=$(realpath $(dirname "$0"))
OUTPUT_FILE=$BASEDIR/main/tzdata.c

cat > "${OUTPUT_FILE}" << EOF
/*
 * THIS FILE IS AUTOGENERATED!
 *
 * See gentzdata.sh
 */

#include "tzdata.h"

const timezone_t timezones[] = {
EOF

timezone_nb=$(find /usr/share/zoneinfo/posix -type f | while read f; do
  if head -n 1 "$f" | grep -q '^TZif2'; then
    echo "    { \"${f#/usr/share/zoneinfo/posix/}\", \"$(tail -n 1 "$f")\" },"
  fi
done | sort | tee -a "${OUTPUT_FILE}" | wc -l)

cat >> "${OUTPUT_FILE}" << EOF
};

const size_t timezones_len = ${timezone_nb};
EOF

echo "Generated ${timezone_nb} timezones in ${OUTPUT_FILE}"

