#!/bin/bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

EMOTIBIT_CSV=""

# Parse required -e or --extension argument
while [[ $# -gt 0 ]]; do
    case "$1" in
        -e|--extension)
            EMOTIBIT_CSV="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 -e|--extension <emotibit_csv_path>"
            exit 1
            ;;
    esac
done

if [[ -z "$EMOTIBIT_CSV" ]]; then
    echo "Error: You must specify -e or --extension <emotibit_csv_path>"
    exit 1
fi

if grep -q ',DO,' "$EMOTIBIT_CSV"; then
    echo "DO found in $EMOTIBIT_CSV"
    exit 1
else
    echo "No DO found in $EMOTIBIT_CSV"
    exit 0
fi