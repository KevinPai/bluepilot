#!/usr/bin/env bash
###############################################################################
# deploy_branch.sh
#
# Deploy a BluePilot branch from your git remote (public or private) onto a
# comma device, build it on-device with SCons, and reboot.
# For a PUBLIC repo no token is needed; --token is only for private repos.
#
# MUST be run ON the comma device (e.g. over SSH). Rationale:
#   - Device binaries must be compiled on the comma device itself
#     (aarch64 + Qualcomm GPU/DSP libs); a PC cannot cross-compile them.
#   - A Windows working tree carries CRLF line endings (no .gitattributes,
#     core.autocrlf=true) that break shell scripts at boot. Letting the device
#     git-checkout the branch produces correct LF line endings.
#
# After this runs once, origin points at your repo, so future updates are pure
# OTA: push a new snapshot to the branch and the device updates itself.
#
# Usage:
#   ./deploy_branch.sh --repo <https-url> [--token <github-token>] [options]
#
# Options:
#   --repo   <url>     Repo URL (https). If omitted, keeps current origin.
#   --token  <token>   GitHub token for PRIVATE repos; omit for public repos.
#   --branch <name>    Branch to deploy (default: bp-6.0).
#   --jobs   <n>       SCons parallel jobs (default: nproc).
#   --no-build         Checkout only; skip the SCons build.
#   --no-reboot        Do not reboot at the end.
#   -y, --yes          Do not prompt; assume yes (reboot automatically).
#   -h, --help         Show this help.
#
# Env var equivalents: BP_PRIVATE_REPO, GITHUB_TOKEN, BP_BRANCH
#
# Examples:
#   # public repo (no token):
#   ./deploy_branch.sh --repo https://github.com/sevenbai/openpilot.git --branch bp-6.0
#   # private repo (token):
#   ./deploy_branch.sh --repo https://github.com/me/private.git --token ghp_xxx --branch bp-6.0
###############################################################################

set -e

###############################################################################
# Color output
###############################################################################
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

print_success() { echo -e "${GREEN}$1${NC}"; }
print_error()   { echo -e "${RED}$1${NC}"; }
print_warning() { echo -e "${YELLOW}$1${NC}"; }
print_info()    { echo -e "${BLUE}$1${NC}"; }

###############################################################################
# Defaults
###############################################################################
TARGET_DIR="/data/openpilot"
BRANCH="${BP_BRANCH:-bp-6.0}"
REPO_URL="${BP_PRIVATE_REPO:-}"
TOKEN="${GITHUB_TOKEN:-}"
JOBS="$(nproc 2>/dev/null || echo 4)"
DO_BUILD=true
DO_REBOOT=true
ASSUME_YES=false

usage() {
    # Print only the leading comment block (skip shebang and pure-divider lines)
    awk 'NR==1{next} /^#/{ if ($0 ~ /^#+$/) next; sub(/^#+ ?/,""); print; next } {exit}' "$0"
    exit "${1:-1}"
}

###############################################################################
# Parse arguments
###############################################################################
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo)     REPO_URL="$2"; shift 2 ;;
        --token)    TOKEN="$2";    shift 2 ;;
        --branch)   BRANCH="$2";   shift 2 ;;
        --jobs)     JOBS="$2";     shift 2 ;;
        --no-build) DO_BUILD=false; shift ;;
        --no-reboot) DO_REBOOT=false; shift ;;
        -y|--yes)   ASSUME_YES=true; shift ;;
        -h|--help)  usage 0 ;;
        *) print_error "Unknown argument: $1"; echo ""; usage 1 ;;
    esac
done

###############################################################################
# Helper: print a remote URL with any embedded credentials masked
###############################################################################
masked_url() {
    echo "$1" | sed -E 's#//[^@/]+@#//***@#'
}

###############################################################################
# Preflight checks
###############################################################################
print_info "=========================================="
print_info "BluePilot private-branch deploy"
print_info "=========================================="
echo ""

# Must run on a comma device
if [ ! -d "/data" ]; then
    print_error "ERROR: /data not found. This script must run ON a comma device."
    exit 1
fi

if [ ! -d "$TARGET_DIR/.git" ]; then
    print_error "ERROR: $TARGET_DIR is not a git checkout."
    print_error "Expected an existing openpilot install at $TARGET_DIR."
    exit 1
fi

cd "$TARGET_DIR"

###############################################################################
# Resolve authenticated remote URL
###############################################################################
if [ -n "$TOKEN" ] && [ -n "$REPO_URL" ]; then
    case "$REPO_URL" in
        https://*@*) ;; # URL already carries credentials, leave as-is
        https://*)  REPO_URL="https://${TOKEN}@${REPO_URL#https://}" ;;
        *) print_warning "TOKEN provided but REPO_URL is not https://, ignoring token" ;;
    esac
fi

###############################################################################
# Configure origin
###############################################################################
if [ -n "$REPO_URL" ]; then
    if git remote get-url origin >/dev/null 2>&1; then
        git remote set-url origin "$REPO_URL"
    else
        git remote add origin "$REPO_URL"
    fi
    print_success "[-] origin set to: $(masked_url "$REPO_URL")"
    if [ -n "$TOKEN" ]; then
        print_warning "    Note: a token in the URL is stored in plaintext in .git/config."
        print_warning "    Use a read-only, single-repo fine-grained token."
    fi
else
    CURRENT_ORIGIN="$(git remote get-url origin 2>/dev/null || echo '(none)')"
    print_info "[-] No --repo given; keeping current origin: $(masked_url "$CURRENT_ORIGIN")"
fi

print_info "[-] Target branch: $BRANCH"
echo ""

###############################################################################
# Fetch & checkout
###############################################################################
print_info "[-] Fetching origin/$BRANCH ..."
if ! git fetch origin "$BRANCH"; then
    print_error "[-] git fetch failed. Check the repo URL / token / branch name / network."
    exit 1
fi

print_info "[-] Checking out $BRANCH ..."
git checkout -f -B "$BRANCH" "origin/$BRANCH"

print_info "[-] Updating submodules ..."
git submodule update --init --recursive

# Remove prebuilt marker so the launcher (and this script) compiles from source
rm -f prebuilt

print_success "[-] Now on $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)"
echo ""

###############################################################################
# AGNOS version check (informational)
###############################################################################
if [ -f /VERSION ] && [ -f launch_env.sh ]; then
    CUR_AGNOS="$(cat /VERSION 2>/dev/null || echo unknown)"
    WANT_AGNOS="$(grep 'export AGNOS_VERSION=' launch_env.sh | head -1 | cut -d'"' -f2)"
    if [ -n "$WANT_AGNOS" ] && [ "$CUR_AGNOS" != "$WANT_AGNOS" ]; then
        print_warning "[-] AGNOS mismatch: device=$CUR_AGNOS, branch requires=$WANT_AGNOS"
        print_warning "    AGNOS will be flashed automatically on the next reboot (extra time + a reboot)."
    else
        print_success "[-] AGNOS OK (device=$CUR_AGNOS)"
    fi
fi
echo ""

###############################################################################
# Disable power save for a faster build (best effort)
###############################################################################
if [ -f "$TARGET_DIR/scripts/disable-powersave.py" ]; then
    python3 "$TARGET_DIR/scripts/disable-powersave.py" 2>/dev/null \
        && print_info "[-] Power save disabled for build" \
        || print_warning "[-] Could not disable power save (continuing)"
fi

###############################################################################
# Build
###############################################################################
if [ "$DO_BUILD" = true ]; then
    print_info "[-] Building with SCons (first build can take 20-40 min) ..."
    export PYTHONPATH="$TARGET_DIR"

    # Remove a stale scons lock if present
    [ -f /data/scons_cache/config.lock ] && rm -f /data/scons_cache/config.lock

    HALF=$(( JOBS / 2 )); [ "$HALF" -lt 1 ] && HALF=1
    BUILD_OK=false
    for N in "$JOBS" "$HALF" 1; do
        print_info "[-]   scons -j$N"
        if scons -j"$N"; then
            BUILD_OK=true
            break
        fi
        print_warning "[-]   build failed at -j$N, retrying with fewer jobs (possible OOM) ..."
    done

    if [ "$BUILD_OK" != true ]; then
        print_error "[-] Build failed. See output above."
        exit 1
    fi
    print_success "[-] Build completed"
else
    print_info "[-] --no-build: skipping SCons (launcher will build on next boot)"
fi
echo ""

###############################################################################
# Reboot
###############################################################################
if [ "$DO_REBOOT" = true ]; then
    if [ "$ASSUME_YES" = true ]; then
        print_info "[-] Rebooting ..."
        sudo reboot
    elif [ -t 0 ]; then
        read -r -p "Reboot now to apply? (yes/no): " REPLY
        if [[ "$REPLY" =~ ^[Yy] ]]; then
            print_info "[-] Rebooting ..."
            sudo reboot
        else
            print_info "[-] Skipped reboot. Run 'sudo reboot' when ready."
        fi
    else
        print_warning "[-] Non-interactive shell; not rebooting automatically."
        print_warning "    Run 'sudo reboot' to apply, or re-run with -y."
    fi
else
    print_info "[-] --no-reboot: done. Run 'sudo reboot' to apply."
fi

print_success "Done."
