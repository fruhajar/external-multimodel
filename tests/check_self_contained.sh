#!/bin/sh
# This library must depend on nothing but glm and the standard library, so that the directory
# can be lifted into a project on its own. Fails on any include that is neither.
set -eu

root="${1:?usage: check_self_contained.sh <library root>}"
own=$(cd "$root" && find include src -name '*.h' | sed 's|.*/||' | sort -u)
bad=0

for header in $(grep -rhoE '^[[:space:]]*#include[[:space:]]*[<"][^>"]+[>"]' \
                    "$root/include" "$root/src" \
                | sed -E 's/.*[<"]([^>"]+)[>"].*/\1/' | sort -u); do
    case "$header" in
        ballistics/mm/*|glm/*) continue ;;
    esac
    base=${header##*/}
    if echo "$own" | grep -qx "$base"; then
        continue
    fi
    # Standard library headers carry no extension.
    case "$header" in
        *.h|*.hpp|*.hh|*/*) echo "not self-contained: $header"; bad=1 ;;
    esac
done

if [ "$bad" -ne 0 ]; then
    exit 1
fi
echo "self-contained: only glm and the standard library"
