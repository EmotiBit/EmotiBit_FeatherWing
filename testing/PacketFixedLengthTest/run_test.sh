#!/bin/bash
set -u

EMOTIBIT_CSV=""

# Parse required -e or --extension argument
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--path)
            EMOTIBIT_CSV="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 -p|--path <emotibit_csv_path>"
            exit 1
            ;;
    esac
done

if [[ -z "$EMOTIBIT_CSV" ]]; then
    echo "Error: You must specify -p or --path <emotibit_csv_path>"
    exit 1
fi

if grep -q ',DO,' "$EMOTIBIT_CSV"; then
    echo "DO found in $EMOTIBIT_CSV"
    exit 1
else
    echo "No DO found in $EMOTIBIT_CSV"
    exit 0
fi