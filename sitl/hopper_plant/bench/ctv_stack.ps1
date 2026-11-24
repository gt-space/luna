<#
.SYNOPSIS
  Starts, stops and inspects the simulated CTV stack from Windows, by running
  ctv_stack.sh inside WSL.

.DESCRIPTION
  The flight computer only builds on Linux, so the stack runs in WSL; this script
  turns Windows paths into WSL paths and calls ctv_stack.sh. Servo listens on
  port 7200, which WSL forwards to Windows, so the GUI connects to 127.0.0.1.

  After "start": open the GUI, connect to servo at 127.0.0.1, pick Umbilical,
  open Views > CTV. One flight per start; "restart" before the next one.

.EXAMPLE
  .\ctv_stack.ps1 start -HopperBuild C:\Users\me\Documents\GitHub\hopper\sims\hopper_6dof_sizer\fsw_bridge\build

.EXAMPLE
  $env:HOPPER_BUILD = 'C:\...\fsw_bridge\build'; .\ctv_stack.ps1 restart

.EXAMPLE
  .\ctv_stack.ps1 status
  .\ctv_stack.ps1 logs fc
  .\ctv_stack.ps1 stop
#>
[CmdletBinding()]
param(
  [Parameter(Position = 0)]
  [ValidateSet('start', 'stop', 'restart', 'status', 'logs', 'gui')]
  [string]$Action = 'start',

  # for logs: servo, ec, bridge or fc
  [Parameter(Position = 1)]
  [string]$Log = 'fc',

  # the hopper repo's fsw_bridge\build folder (default: the HOPPER_BUILD environment variable)
  [string]$HopperBuild = $env:HOPPER_BUILD,

  # a plant folder other than the default (plant_strut, plant, verify\plant under HopperBuild)
  [string]$PlantCode = $env:HOPPER_PLANT_CODE,

  # the WSL distribution (default: your default distribution)
  [string]$Distro = '',

  # fly on the plant's true state instead of the navigation filter
  [switch]$TruthNav,

  # do not run cargo/make first
  [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
  throw 'wsl.exe not found: the simulated stack runs in WSL (install it with: wsl --install)'
}

$wsl = @()
if ($Distro) { $wsl += @('-d', $Distro) }

# A Windows path as WSL sees it
function ConvertTo-WslPath([string]$Path) {
  # forward slashes: wsl.exe hands the command to a Linux shell, which eats backslashes
  $full = (Resolve-Path -LiteralPath $Path).Path.Replace('\', '/')
  $out = & wsl.exe @wsl -- wslpath -a "'$full'"
  if ($LASTEXITCODE -ne 0 -or -not $out) { throw "wslpath could not convert $Path" }
  return ($out | Select-Object -First 1).Trim()
}

$script = ConvertTo-WslPath (Join-Path $PSScriptRoot 'ctv_stack.sh')

# The environment the script reads, passed through `env` so nothing depends on the WSL profile
$envArgs = @()
if ($Action -in @('start', 'restart')) {
  if (-not $HopperBuild) {
    throw 'give -HopperBuild <hopper>\sims\hopper_6dof_sizer\fsw_bridge\build (or set $env:HOPPER_BUILD)'
  }
  $envArgs += "HOPPER_BUILD=$(ConvertTo-WslPath $HopperBuild)"
  if ($PlantCode) { $envArgs += "HOPPER_PLANT_CODE=$(ConvertTo-WslPath $PlantCode)" }
  if ($TruthNav) { $envArgs += 'SIL_TRUTH_NAV=1' }
  if ($SkipBuild) { $envArgs += 'SKIP_BUILD=1' }
}

$argsList = @($Action)
if ($Action -eq 'logs') { $argsList += $Log }

& wsl.exe @wsl -- env @envArgs bash $script @argsList
exit $LASTEXITCODE
