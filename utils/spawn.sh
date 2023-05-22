#!/usr/bin/env bash
set -e

gz service -s "/world/default/create" \
    --reqtype "gz.msgs.EntityFactory" \
    --reptype "gz.msgs.Boolean" \
    --timeout 1000 \
    --req "sdf_filename: \"$1\" pose: {position{x: 0 y: 0 z: 0}}"

