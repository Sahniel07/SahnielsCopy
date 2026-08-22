#!/usr/bin/env bash
# Build and run each arithmetic configuration, first without
# nondimensionalization and then with it.
#
#   ./sweep.sh              run the sweep
#   ./sweep.sh --dry-run    list the configurations, build nothing
#
# Each run writes its CSVs to sweep_build/ and its plot to media/, both named
# after the format, so runs do not overwrite each other.
set -u
cd "$(dirname "$0")"

BUILD=sweep_build
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

# label            posit  bits  es      ("-" = not a posit run)
CONFIGS=(
"half16         OFF    16    -"
"posit16es0     ON     16    0"
"posit16es1     ON     16    1"
"posit16es2     ON     16    2"
"posit16es3     ON     16    3"
"ieee32         OFF    32    -"
"posit32es2     ON     32    0"
"posit32es3     ON     32    1"
"posit32es2     ON     32    2"
"posit32es3     ON     32    3"
"fp64           OFF    64    -"
)

total=$(( ${#CONFIGS[@]} * 2 ))
n=0

for QUANT in OFF ON; do
    echo ""
    echo "================ nondimensionalization: $QUANT ================"

    for cfg in "${CONFIGS[@]}"; do
        set -- $cfg
        LABEL=$1; POSIT=$2; BITS=$3; ES=$4
        n=$((n + 1))

        ES_ARG=""
        [ "$ES" != "-" ] && ES_ARG="-DPOSIT_ES=$ES"

        printf "[%2d/%2d] %-12s bits=%-2s es=%-2s quant=%-3s " \
               "$n" "$total" "$LABEL" "$BITS" "$ES" "$QUANT"

        if [ "$DRY_RUN" -eq 1 ]; then
            echo "(dry run)"
            continue
        fi

        if ! cmake -S . -B "$BUILD" \
                -DENABLE_POSIT="$POSIT" -DREAL_BITS="$BITS" \
                -DENABLE_QUANTIZATION="$QUANT" $ES_ARG > /dev/null 2>&1; then
            echo "CONFIGURE FAILED"
            continue
        fi

        if ! cmake --build "$BUILD" -j"$(nproc)" > /dev/null 2>&1; then
            echo "BUILD FAILED"
            continue
        fi

        ( cd "$BUILD" && ./dynamic_game_trajectory_planner > /dev/null 2>&1 )
        if [ $? -eq 0 ]; then
            echo "done"
        else
            echo "RUN FAILED"
        fi
    done
done

echo ""
echo "sweep finished - CSVs in $BUILD/, plots in media/"
