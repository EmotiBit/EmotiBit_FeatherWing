# GitHub Workflows and Bash Scripts Documentation

This document provides a comprehensive overview of all GitHub workflows and bash scripts in the EmotiBit FeatherWing repository.

## Table of Contents
- [GitHub Workflows](#github-workflows)
  - [Development Workflow: Continuous Build](#development-workflow-continuous-build)
  - [Release Workflow: Create Release](#release-workflow-create-release)
- [Bash Scripts](#bash-scripts)
  - [CI/CD Scripts](#cicd-scripts)
  - [Development Scripts](#development-scripts)
  - [Test Scripts](#test-scripts)
- [Development Process](#development-process)
- [Dependency Management Approaches](#dependency-management-approaches)
- [Configuration Files](#configuration-files)
- [Requirements Summary](#requirements-summary)
- [Maintenance Notes](#maintenance-notes)

---

## GitHub Workflows

The repository uses two GitHub Actions workflows to automate the build and release process. These workflows are defined in `.github/workflows/`.

### Development Workflow: Continuous Build
**File:** `.github/workflows/build-and-upload-binaries.yml`

**When it runs:** Automatically on every push to any branch

**What it does:** Provides continuous integration by building firmware binaries for every commit, ensuring code changes compile successfully.

**For developers:**
- Push to any branch triggers an automatic build
- Check the Actions tab to see build status
- Build artifacts (`.bin` and `.hex` files) are available for download for 30 days
- Artifacts include both firmware variants plus a dependency report
- Artifact naming: `emotibit-firmware-{version}` (version from `library.properties`)

**Why it's useful:**
- Validates that changes compile before merging
- Provides downloadable binaries for testing without local builds
- Generates a dependency snapshot for each build
- Ensures all branches maintain buildable state

**Technical notes:**
- Uses `download_dependencies.sh`, `generate_dependency_report.sh`, and `build.sh`
- Automatically cleans up build environment after completion

---

### Release Workflow: Create Release
**File:** `.github/workflows/create-release.yml`

**When it runs:** Manually triggered via GitHub Actions UI

**What it does:** Creates a GitHub release using binaries from the latest successful dev branch build.

**For developers - How to create a release:**

1. Ensure the `dev` branch has a successful build
2. Go to Actions → "Create Release from Latest CI Build" → "Run workflow"
3. Optionally provide:
   - Custom release name (defaults to `v{version}` from library.properties)
   - Pre-release flag (for beta/RC releases)
4. Click "Run workflow"
5. Workflow creates a **draft release** - review it before publishing

**What gets included:**
- Stock firmware binaries (`.bin` and `.hex` files) from both variants
- Auto-generated release notes with:
  - Link to the source build workflow run
  - Library dependency versions and git commits

**Important constraints:**
- Only creates releases from the `dev` branch
- Requires at least one successful build on `dev`
- Releases are created as drafts for manual review
- Version tag format: `v{version}` (e.g., `v1.14.3`)

**Best practices:**
- Always verify the version number in `library.properties` before creating a release
- Review the draft release before publishing
- Use pre-release flag for beta/testing versions
- Ensure changelog/release notes are updated before publishing

---

## Bash Scripts

### CI/CD Scripts

These scripts are used by the GitHub workflows for automated builds and releases.

#### download_dependencies.sh
**Location:** `./download_dependencies.sh`

**Purpose:** Downloads all project dependencies based on `library.properties` and `depends_urls.json`.

**How it works:**
1. Reads dependency list from `library.properties`
2. Gets GitHub repository URLs from `depends_urls.json`
3. Parses dependency names and version specifications
4. Clones each dependency repository at the specified version
5. Downloads to the parent directory (same level as FeatherWing)

**Features:**
- Supports version pinning with format: `Name (=version)`
- Tries both `v{version}` and `{version}` tag formats
- Falls back to default branch if version tag not found
- Skips already downloaded repositories
- Shallow clone (--depth 1) for efficiency

**Requirements:**
- `jq` command-line JSON processor
- Git

**Exit codes:**
- 0: Success
- 1: Missing required files or tools

---

#### generate_dependency_report.sh
**Location:** `./generate_dependency_report.sh`

**Purpose:** Generates a text report of all dependency versions and git commits.

**Output:** `dependency_report.txt` in the script directory

**Report format:**
```
EmotiBit FeatherWing Dependency Report
Generated: [timestamp]

LibraryName1: version (git_commit_hash)
LibraryName2: version (git_commit_hash)
...
```

**How it works:**
1. Reads dependencies from `library.properties`
2. Gets repository information from `depends_urls.json`
3. For each dependency:
   - Locates the downloaded repository
   - Retrieves the current git commit hash
   - Writes to report file

**Requirements:**
- `jq` command-line JSON processor
- Dependencies must be already downloaded

---

#### build.sh
**Location:** `./build.sh`

**Purpose:** Builds all EmotiBit FeatherWing firmware variants using PlatformIO.

**Builds:**
1. EmotiBit stock firmware (`EmotiBit_stock_firmware/`)
2. EmotiBit stock firmware PPG 100Hz (`EmotiBit_stock_firmware_PPG_100Hz/`)

**How it works:**
1. Checks if PlatformIO is installed
2. Changes to each firmware directory
3. Runs `pio run` to build the firmware
4. Reports completion

**Requirements:**
- PlatformIO CLI (`pio` command)
- All dependencies must be downloaded

**Exit codes:**
- 0: Success (all builds completed)
- 1: PlatformIO not installed or build failed

---

### Development Scripts

These scripts are for local development and manual dependency management.

#### CloneEmotiBitFW.sh
**Location:** `./CloneEmotiBitFW.sh`

**Purpose:** Clones all EmotiBit dependency repositories using SSH.

**How it works:**
1. Calls `ExtractDepends.sh` to generate dependency list
2. Starts SSH agent and adds SSH key (`~/.ssh/id_ed25519`)
3. Reads repository names from `EmotiBit_FeatherWing_depends.txt`
4. Clones each repository from GitHub using SSH
5. Clones to parent directory (one level up from current)
6. Skips repositories that already exist

**Requirements:**
- SSH key configured for GitHub (`~/.ssh/id_ed25519`)
- SSH access to EmotiBit GitHub repositories

**Note:** This is a legacy script. The newer `download_dependencies.sh` is preferred for CI/CD as it uses HTTPS and supports version pinning.

---

#### CheckoutMasterEmotiBitFW.sh
**Location:** `./CheckoutMasterEmotiBitFW.sh`

**Purpose:** Checks out the master branch for all dependency repositories.

**How it works:**
1. Calls `ExtractDepends.sh` to generate dependency list
2. Starts SSH agent and adds SSH key
3. Iterates through each repository
4. Changes to repository directory and runs `git checkout master`

**Requirements:**
- SSH key configured for GitHub
- Repositories must be already cloned

**Use case:** Useful for ensuring all dependencies are on the master branch for development or testing.

---

#### ExtractDepends.sh
**Location:** `./ExtractDepends.sh`

**Purpose:** Extracts dependency names from `library.properties` and writes them to a text file.

**Output:** `EmotiBit_FeatherWing_depends.txt`

**How it works:**
1. Deletes existing `EmotiBit_FeatherWing_depends.txt` if present
2. Reads `library.properties` line by line
3. Finds the line starting with `depends=`
4. Parses comma-separated dependency list
5. Converts spaces to underscores in repository names
6. Writes each repository name to the output file

**Note:** This script is called by other development scripts (`CloneEmotiBitFW.sh`, `CheckoutMasterEmotiBitFW.sh`, `UpdateEmotiBitFW.sh`) to generate the repository list.

---

#### UpdateEmotiBitFW.sh
**Location:** `./UpdateEmotiBitFW.sh`

**Purpose:** Updates all dependency repositories by pulling latest changes.

**How it works:**
1. Calls `ExtractDepends.sh` to generate dependency list
2. Starts SSH agent and adds SSH key
3. Iterates through each repository
4. Changes to repository directory and runs `git pull`

**Requirements:**
- SSH key configured for GitHub
- Repositories must be already cloned
- No uncommitted changes in repositories (to avoid conflicts)

**Use case:** Quick way to update all dependencies to their latest versions.

## Dependency Management Approaches

The repository uses two different approaches for dependency management:

### Modern Approach (CI/CD)
- **Script:** `download_dependencies.sh`
- **Configuration:** `library.properties` + `depends_urls.json`
- **Method:** HTTPS cloning with version pinning
- **Advantages:**
  - No SSH key required
  - Version control per dependency
  - Works in CI/CD environments
  - Follows Arduino library conventions

### Legacy Approach (Local Development)
- **Scripts:** `CloneEmotiBitFW.sh`, `CheckoutMasterEmotiBitFW.sh`, `UpdateEmotiBitFW.sh`
- **Configuration:** `library.properties` + `ExtractDepends.sh`
- **Method:** SSH cloning without version pinning
- **Advantages:**
  - Quick updates with git pull
  - Works with existing SSH credentials
  - Simple for local development

**Recommendation:** Use the modern approach (`download_dependencies.sh`) for reproducible builds and CI/CD. The legacy scripts remain useful for rapid local development iterations.

---

## Development Process

### Typical Development Lifecycle

**Feature Development:**
```
1. Create feature branch from dev
2. Make code changes
3. Push to GitHub → automatic build runs
4. Download artifacts from Actions tab to test
5. Iterate on changes (each push triggers new build)
6. Create pull request to dev
7. Merge after review and successful build
```

**Preparing a Release:**
```
1. Update version in library.properties and merge featire branch to dev branch 
2. Push changes → automatic build runs
3. Verify build succeeds on dev
4. Manually trigger "Create Release" workflow
5. Review draft release:
   - Check binaries are correct
   - Review auto-generated dependency information
   - Add/edit release notes as needed
6. Publish release when ready
```