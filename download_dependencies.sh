#!/bin/bash

# Script to download all dependencies using library.properties for names/versions and dependencies.json for URLs
# Downloads dependencies at the same level as the FeatherWing directory
# Usage: ./download_dependencies.sh

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIBRARY_PROPERTIES="$SCRIPT_DIR/library.properties"
DEPS_JSON="$SCRIPT_DIR/depends_urls.json"
# Libraries will be downloaded one level up from FeatherWing (same level as FeatherWing)
LIBS_DIR="$(dirname "$SCRIPT_DIR")"

# Check if library.properties exists
if [ ! -f "$LIBRARY_PROPERTIES" ]; then
    echo "Error: library.properties not found in $SCRIPT_DIR"
    exit 1
fi

# Check if depends_urls.json exists
if [ ! -f "$DEPS_JSON" ]; then
    echo "Error: depends_urls.json not found in $SCRIPT_DIR"
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
        version=""
    fi
    
    # Get GitHub URL from depends_urls.json
    github=$(jq -r --arg name "$name" '.library_urls[$name] // empty' "$DEPS_JSON")
    
    if [ -z "$github" ]; then
        echo "Warning: No URL found for library '$name' in depends_urls.json, skipping..."
        continue
    fi
    
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
            
            # Try cloning with v prefix first
            if git clone --depth 1 --branch "v$version" --single-branch "$github" "$lib_path" 2>/dev/null; then
                echo "  Cloned at tag: v$version"
            # Try cloning without v prefix
            elif git clone --depth 1 --branch "$version" --single-branch "$github" "$lib_path" 2>/dev/null; then
                echo "  Cloned at tag: $version"
            else
                echo "  Warning: Specified version tags 'v$version' and '$version' not found, cloning default branch"
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