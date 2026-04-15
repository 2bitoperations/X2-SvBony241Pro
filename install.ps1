# install.ps1 — Install the SVBony SV241 Pro plugin for TheSkyX on Windows
#
# Run from the folder containing the plugin files (right-click → Run with PowerShell,
# or from a terminal: .\install.ps1)
#
# Usage:
#   .\install.ps1              # install
#   .\install.ps1 -Uninstall  # remove installed files

param([switch]$Uninstall)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$LIB = "libx2svbony241pro.dll"
$UI  = "sv241pro.ui"
$LST = "powercontrollist Svbony.txt"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Locate TheSkyX — check the two standard Program Files roots
$candidates = @(
    "$env:ProgramFiles\Software Bisque\TheSkyX Professional Edition",
    "${env:ProgramFiles(x86)}\Software Bisque\TheSkyX Professional Edition"
)

$tsxRoot = $null
foreach ($c in $candidates) {
    if (Test-Path $c) { $tsxRoot = $c; break }
}

if (-not $tsxRoot) {
    Write-Host ""
    Write-Host "ERROR: TheSkyX installation not found in standard locations." -ForegroundColor Red
    Write-Host "       Edit the `$candidates list in this script to point at your install." -ForegroundColor Red
    exit 1
}

$pluginDir = Join-Path $tsxRoot "Resources\Common\PlugIns\PowerControlPlugIns"
$listDir   = Join-Path $tsxRoot "Resources\Common\Miscellaneous Files"

# ---------------------------------------------------------------------------
# Uninstall
# ---------------------------------------------------------------------------
if ($Uninstall) {
    Write-Host "Uninstalling SVBony SV241 Pro plugin..."
    foreach ($f in @("$pluginDir\$LIB", "$pluginDir\$UI", "$listDir\$LST")) {
        if (Test-Path $f) { Remove-Item $f; Write-Host "  Removed $f" }
    }
    Write-Host "Done."
    exit 0
}

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
foreach ($dir in @($pluginDir, $listDir)) {
    if (-not (Test-Path $dir)) {
        Write-Host ""
        Write-Host "ERROR: Directory not found: $dir" -ForegroundColor Red
        Write-Host "       Is TheSkyX installed?" -ForegroundColor Red
        exit 1
    }
}

if (-not (Test-Path (Join-Path $ScriptDir $LIB))) {
    Write-Host ""
    Write-Host "ERROR: $LIB not found next to this script." -ForegroundColor Red
    exit 1
}

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
Write-Host "Installing plugin..."

Copy-Item (Join-Path $ScriptDir $LIB) "$pluginDir\$LIB" -Force
Write-Host "  -> $pluginDir\$LIB"

Copy-Item (Join-Path $ScriptDir $UI) "$pluginDir\$UI" -Force
Write-Host "  -> $pluginDir\$UI"

Copy-Item (Join-Path $ScriptDir $LST) "$listDir\$LST" -Force
Write-Host "  -> $listDir\$LST"

Write-Host ""
Write-Host "Installation complete." -ForegroundColor Green
Write-Host "Restart TheSkyX and select 'SVBony SV241 Pro' from Telescope > Power Control Box."
