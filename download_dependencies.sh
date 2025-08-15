#!/bin/bash

# Script to download all dependencies from dependencies.json and checkout specified versions
# Downloads dependencies at the same level as the FeatherWing directory
# Usage: ./download_dependencies.sh

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_JSON="$SCRIPT_DIR/dependencies.json"
# Libraries will be downloaded one level up from FeatherWing (same level as FeatherWing)
LIBS_DIR="$(dirname "$SCRIPT_DIR")"

# Check if dependencies.json exists
if [ ! -f "$DEPS_JSON" ]; then
    echo "Error: dependencies.json not found in $SCRIPT_DIR"
    exit 1
fi

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: jq is required but not installed. Please install jq first."
    echo "  On macOS: brew install jq"
    echo "  On Ubuntu/Debian: sudo apt-get install jq"
    exit 1
fi

echo "Starting dependency download..."
echo "Libraries will be downloaded to: $LIBS_DIR"
echo ""

# Read dependencies from JSON and process each one
jq -r '.dependencies[] | @base64' "$DEPS_JSON" | while read -r data; do
    # Decode the JSON object
    dep=$(echo "$data" | base64 --decode)
    
    name=$(echo "$dep" | jq -r '.name')
    github=$(echo "$dep" | jq -r '.github')
    version=$(echo "$dep" | jq -r '.version')
    
    echo "Processing: $name"
    echo "  Repository: $github"
    echo "  Version: $version"
    
    # Extract repository name from GitHub URL
    repo_name=$(basename "$github" .git)
    lib_path="$LIBS_DIR/$repo_name"
    
    # Clone repository
    if [ -d "$lib_path" ]; then
        echo "  Repository already exists, skipping..."
    else
        if [ -n "$version" ] && [ "$version" != "" ]; then
            echo "  Cloning repository at version: $version"
            
            # Try cloning with version tag first, then with v prefix
            if git clone --depth 1 --branch "$version" --single-branch "$github" "$lib_path" 2>/dev/null; then
                echo "  ✓ Cloned at tag: $version"
            elif git clone --depth 1 --branch "v$version" --single-branch "$github" "$lib_path" 2>/dev/null; then
                echo "  ✓ Cloned at tag: v$version"
            else
                echo "  ⚠ Warning: Version tag '$version' or 'v$version' not found, cloning default branch"
                git clone --depth 1 "$github" "$lib_path"
            fi
        else
            echo "  Cloning repository (default branch)..."
            git clone --depth 1 "$github" "$lib_path"
            echo "  ℹ No version specified, cloned default branch"
        fi
    fi
    
    echo "  ✓ Done"
    echo ""
    
    cd "$SCRIPT_DIR"
done

echo "All dependencies have been processed!"
echo "Libraries are available in: $LIBS_DIR"