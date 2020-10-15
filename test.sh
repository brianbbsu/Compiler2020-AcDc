#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

TEST_DIR="./test"
EXE="./src/AcDc"

TEST_NAME_WIDTH=30
TEST_STATUS_WIDTH=30
FMT="%-${TEST_NAME_WIDTH}s %-${TEST_STATUS_WIDTH}s %s\n"

echo "======"
echo "Running tests for Ac-Dc compiler"
echo "Compiler: `realpath $EXE`"
echo "Test DIR: `realpath $TEST_DIR`"
echo "======"

SOURCES=`find "$TEST_DIR" -name '*.ac' | sort -V`
WORKSPACE=`mktemp -d`

printf "$FMT" "Testcase" "Status" "Test Flags"

SUCCESS_CNT=0
TOTAL_CNT=0
for input in $SOURCES; do
    TOTAL_CNT=$((TOTAL_CNT+1))
    REGEX='.*\/([^\/.]+)+(\.error)?(\.DCerror)?\.ac'
    if [[ ! $input =~ $REGEX ]]; then
        echo "Regex match failed for input file ${input}"
        exit 2
    fi
    testcase="${BASH_REMATCH[1]}"
    ERROR_FLAG=0
    DCERROR_FLAG=0
    EXPECTED_FLAG=0
    for flag in ${BASH_REMATCH[@]:2}; do
        if [[ -n "$flag" ]]; then
            case "$flag" in
              .error)
                ERROR_FLAG=1
                ;;
              .DCerror)
                DCERROR_FLAG=1
                ;;
              *)
                echo "Flag '$flag' was not properly hanlded."
                exit 2
                ;;
            esac
        fi
    done
    expected_output="`dirname $input`/$testcase.expected"
    if [[ -f "$expected_output" ]]; then
        EXPECTED_FLAG=1
    fi
    flags=""
    if [[ "$ERROR_FLAG" = "1" ]]; then flags="${flags},error"; fi
    if [[ "$DCERROR_FLAG" = "1" ]]; then flags="${flags},DCerror"; fi
    if [[ "$EXPECTED_FLAG" = "1" ]]; then flags="${flags},expected"; fi
    flags="${flags#,}"
    EXIT_STATUS=0
    { "$EXE" "$input" "${WORKSPACE}/output.dc" 2>"${WORKSPACE}/error" >/dev/null \
        || EXIT_STATUS=$? ; } 2>/dev/null
    if [[ $EXIT_STATUS -gt 1 ]]; then
        # Runtime Error
        printf "$FMT" "$testcase" "Runtime Error" "$flags"
        continue
    elif [[ "$EXIT_STATUS" != "$ERROR_FLAG" ]]; then
        if [[ "$ERROR_FLAG" = "0" ]]; then
            printf "$FMT" "$testcase" "Failed - Compile Error" "$flags"
        else
            printf "$FMT" "$testcase" "Failed - Compile Passed" "$flags"
        fi
        continue
    fi
    dc -f "${WORKSPACE}/output.dc" > "${WORKSPACE}/dc.out" 2>"${WORKSPACE}/dc.err"
    if [[ -s "${WORKSPACE}/dc.err" ]]; then
        if [[ "$DCERROR_FLAG" -eq "0" ]]; then
            printf "$FMT" "$testcase" "Failed - dc Runtime Error" "$flags"
            continue
        fi
    else
        if [[ "$DCERROR_FLAG" -eq "1" ]]; then
            printf "$FMT" "$testcase" "Failed - dc Passed" "$flags"
            continue
        fi
    fi
    if [[ "$EXPECTED_FLAG" = "1" ]] && \
        ! `diff --strip-trailing-cr -B -Z "${WORKSPACE}/dc.out" \
            "$expected_output" >/dev/null 2>&1`; then
        printf "$FMT" "$testcase" "Failed - Wrong dc Output" "$flags"
        continue
    fi
    printf "$FMT" "$testcase" "Success" "$flags"
    SUCCESS_CNT=$((SUCCESS_CNT+1))
done
echo "======"
echo "Done! ${SUCCESS_CNT} Passed, $((TOTAL_CNT-SUCCESS_CNT)) Failed, ${TOTAL_CNT} Total"
echo "======"

rm -rf "$WORKSPACE"

if [[ $SUCCESS_CNT -ne $TOTAL_CNT ]]; then
    exit 1
else
    exit 0
fi
