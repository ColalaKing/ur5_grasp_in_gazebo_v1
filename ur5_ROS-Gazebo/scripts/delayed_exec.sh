#!/usr/bin/env bash
set -e

delay="$1"
shift

sleep "$delay"
exec "$@"
