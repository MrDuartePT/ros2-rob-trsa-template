#!/bin/bash
set -e

if [ "$MACOS_BUILD" = "true" ]; then
    exec /usr/local/share/desktop-init.sh "$@"
else
    exec "$@"
fi
