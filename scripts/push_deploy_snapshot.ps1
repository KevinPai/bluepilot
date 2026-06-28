<#
.SYNOPSIS
    Push a history-free snapshot of the current tree to a private deploy repo.

.DESCRIPTION
    The BluePilot history contains Git LFS objects. Pushing the full branch to a
    fresh private repo fails: GitHub's pre-receive hook rejects it with GH008
    ("unknown Git LFS objects") because the LFS blobs aren't uploaded.

    The comma device only needs the branch *tip*, and the tip has no LFS files,
    so this script pushes an orphan (parentless) commit containing just the
    current tree. No LFS, no GH008, small and fast. The device's git checkout /
    OTA updater force-checkouts, so replacing the remote branch each time is fine.

.PARAMETER Remote
    Name of the git remote pointing at your PRIVATE repo. Default: mine

.PARAMETER Branch
    Remote branch to (force) update with the snapshot. Default: bp-6.0

.PARAMETER SourceRef
    Which local ref's tree to snapshot. Default: HEAD

.PARAMETER Message
    Commit message for the snapshot. Default: auto (tag + source short SHA + date)

.PARAMETER DryRun
    Build the snapshot commit locally but do NOT push.

.EXAMPLE
    .\scripts\push_deploy_snapshot.ps1
    Snapshot HEAD and force-push to mine/bp-6.0.

.EXAMPLE
    .\scripts\push_deploy_snapshot.ps1 -Branch bp-6.0 -Message "fix nose-dive"
#>
[CmdletBinding()]
param(
    [string]$Remote    = "mine",
    [string]$Branch    = "bp-6.0",
    [string]$SourceRef = "HEAD",
    [string]$Message   = "",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Write-Info  ($m) { Write-Host $m -ForegroundColor Blue }
function Write-Ok    ($m) { Write-Host $m -ForegroundColor Green }
function Write-Warn  ($m) { Write-Host $m -ForegroundColor Yellow }
function Write-Err   ($m) { Write-Host $m -ForegroundColor Red }

# Must be inside a git work tree
git rev-parse --is-inside-work-tree *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Err "ERROR: not inside a git repository."
    exit 1
}

# Remote must exist
$remoteUrl = git remote get-url $Remote 2>$null
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($remoteUrl)) {
    Write-Err "ERROR: remote '$Remote' not found."
    Write-Warn "Add it first, e.g.:"
    Write-Warn "  git remote add $Remote https://github.com/<you>/bluepilot-private.git"
    exit 1
}
# Mask any embedded credentials when displaying the URL
$maskedUrl = $remoteUrl -replace '//[^@/]+@', '//***@'

# Resolve the source commit (for traceability in the message)
$srcSha = (git rev-parse --short $SourceRef).Trim()
if ($LASTEXITCODE -ne 0) {
    Write-Err "ERROR: cannot resolve source ref '$SourceRef'."
    exit 1
}

if ([string]::IsNullOrWhiteSpace($Message)) {
    $stamp   = Get-Date -Format "yyyy-MM-dd HH:mm"
    $Message = "$Branch deploy snapshot | src $srcSha | $stamp"
}

Write-Info "=========================================="
Write-Info "Push deploy snapshot"
Write-Info "=========================================="
Write-Info "  Remote : $Remote ($maskedUrl)"
Write-Info "  Branch : $Branch"
Write-Info "  Source : $SourceRef ($srcSha)"
Write-Info "  Message: $Message"
Write-Host ""

# Create an orphan (parentless) commit from the source tree only.
# Quoting keeps PowerShell from mangling ^{} in the rev-parse expression.
$snap = (git commit-tree "$SourceRef^{tree}" -m $Message).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($snap)) {
    Write-Err "ERROR: failed to create snapshot commit."
    exit 1
}
Write-Ok "[-] Snapshot commit: $snap"

# Sanity check: the snapshot must contain no LFS pointer files
$lfsCount = (git lfs ls-files $snap 2>$null | Measure-Object -Line).Lines
if ($lfsCount -gt 0) {
    Write-Warn "[-] WARNING: snapshot contains $lfsCount LFS file(s); push may hit GH008."
} else {
    Write-Ok "[-] Snapshot is LFS-free (good)"
}

if ($DryRun) {
    Write-Warn "[-] --DryRun: not pushing. To push manually:"
    Write-Host "    git push $Remote --force `"${snap}:refs/heads/$Branch`""
    exit 0
}

Write-Info "[-] Pushing to $Remote/$Branch (force) ..."
git push $Remote --force "${snap}:refs/heads/$Branch"
if ($LASTEXITCODE -ne 0) {
    Write-Err "[-] Push failed."
    exit 1
}

Write-Ok "Done. $Remote/$Branch now points at snapshot $snap (src $srcSha)."
Write-Info "On the device: pull/checkout $Branch and rebuild (see scripts/deploy_branch.sh)."
