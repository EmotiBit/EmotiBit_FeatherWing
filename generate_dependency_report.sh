#!/bin/bash

# Script to generate a report of dependency versions and git commits
# Usage: ./generate_dependency_report.sh

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIBRARY_PROPERTIES="$SCRIPT_DIR/library.properties"
DEPS_JSON="$SCRIPT_DIR/depends_urls.json"
LIBS_DIR="$(dirname "$SCRIPT_DIR")"
REPORT_FILE="$SCRIPT_DIR/dependency_report.txt"

# Check if required files exist
if [ ! -f "$LIBRARY_PROPERTIES" ]; then
    echo "Error: library.properties not found in $SCRIPT_DIR"
    exit 1
fi

if [ ! -f "$DEPS_JSON" ]; then
    echo "Error: depends_urls.json not found in $SCRIPT_DIR"
    exit 1
fi

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: jq is required but not installed. Please install jq first."
    exit 1
fi

echo "Generating dependency report..."

# Create report header
cat > "$REPORT_FILE" << EOF
EmotiBit FeatherWing Dependency Report
Generated: $(date)

EOF

# Parse dependencies from library.properties
depends_line=$(grep "^depends=" "$LIBRARY_PROPERTIES" | cut -d'=' -f2-)

# Split dependencies by comma and process each one
IFS=',' read -ra DEPS <<< "$depends_line"
for dep in "${DEPS[@]}"; do
    # Trim whitespace and extract name and version
    dep=$(echo "$dep" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
    
    # Check if dependency has version specification like "Name (=version)"
    if [[ "$dep" == *"(="* ]]; then
        name=$(echo "$dep" | sed 's/[[:space:]]*(.*//')
        version=$(echo "$dep" | sed 's/.*(\=//' | sed 's/).*//')
    else
        name="$dep"
        version="latest"
    fi
    
    # Get GitHub URL from depends_urls.json
    github=$(jq -r --arg name "$name" '.library_urls[$name] // empty' "$DEPS_JSON")
    
    if [ -z "$github" ]; then
        continue
    fi
    
    # Extract repository name from GitHub URL
    repo_name=$(basename "$github" .git)
    lib_path="$LIBS_DIR/$repo_name"
    
    # Check if library directory exists and get git commit
    if [ -d "$lib_path" ] && [ -d "$lib_path/.git" ]; then
        cd "$lib_path"
        git_commit=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
        cd "$SCRIPT_DIR"
        
        echo "$name: $version ($git_commit)" >> "$REPORT_FILE"
    else
        echo "$name: $version (not found)" >> "$REPORT_FILE"
    fi
done

echo "Dependency report generated: $REPORT_FILE"