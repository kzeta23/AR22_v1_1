# Flash build/AR22_LCU_v1_0F.hex to the target via ST-Link (STM32CubeProgrammer CLI).
#
# Robust probe + connect-mode selection:
#   - Enumerates every connected ST-Link probe.
#   - For each probe, tries connect modes in order: UR (Under Reset) then HotPlug.
#     This board does NOT answer SWD in HotPlug/Normal mode and needs Under Reset,
#     so UR is tried first; HotPlug is kept as a fallback for other boards.
#   - The first probe+mode that actually reaches the STM32 target is used to program,
#     and the SAME mode is used for the download (plain Normal mode fails on this board).
#   - Set $env:STLINK_SN to force a specific probe (it is still mode-probed UR->HotPlug).
#
# NOTE on this rig: two STLINK-V3MINIE probes are present and USB enumeration is flaky
# (often only one shows in --list at a time). Trying all listed SNs + UR handles the
# common case. If the wrong probe is the only one enumerated, replug or set $env:STLINK_SN.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot\..").Path

$cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
if (-not (Test-Path $cli)) {
  $found = Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue
  if ($found) { $cli = $found.Source } else { throw "STM32_Programmer_CLI not found" }
}

$hex = "$root\build\AR22_LCU_v1_0F.hex"
if (-not (Test-Path $hex)) { throw "$hex not found - build first" }

# Connect modes to try, in priority order. UR works on this board; HotPlug is a fallback.
$modes = @('UR', 'HotPlug')

# --- Build the candidate probe SN list ---
$sns = @()
if ($env:STLINK_SN) {
  $sns = @($env:STLINK_SN)
  Write-Host "Using ST-Link SN from STLINK_SN: $($env:STLINK_SN)"
} else {
  # NOTE: do not use '2>&1' on the native exe in Windows PowerShell 5.1 (it mangles stdout).
  $listOut = (& $cli --list | Out-String)
  foreach ($m in [regex]::Matches($listOut, 'ST-LINK SN\s*:\s*(\S+)')) { $sns += $m.Groups[1].Value }
  $sns = $sns | Select-Object -Unique
  if ($sns.Count -eq 0) { throw "No ST-Link probe detected. Connect the probe wired to the board's SWD." }
  Write-Host ("Detected ST-Link probe(s): {0}" -f ($sns -join ', '))
}

# --- Find a probe+mode that actually reaches an STM32 target ---
$targetSN   = $null
$targetMode = $null
foreach ($sn in $sns) {
  foreach ($mode in $modes) {
    Write-Host "Trying probe $sn  mode=$mode ..."
    & $cli -c port=SWD mode=$mode freq=4000 sn=$sn | Out-Null
    if ($LASTEXITCODE -eq 0) { $targetSN = $sn; $targetMode = $mode; break }
  }
  if ($targetSN) { break }
}
if (-not $targetSN) {
  throw ("No connected probe reached an STM32 target (tried SNs [{0}], modes [{1}]). " -f ($sns -join ', '), ($modes -join ', ')) +
        "Check SWD wiring/power, connect the ST-Link wired to the board's SWD, or set `$env:STLINK_SN."
}
Write-Host "Selected ST-Link SN: $targetSN  (connect mode: $targetMode)"

# --- Program + verify + reset, using the mode that connected ---
& $cli -c port=SWD freq=4000 sn=$targetSN mode=$targetMode -d $hex -v -rst
if ($LASTEXITCODE -ne 0) { throw "Flash failed (CLI exit $LASTEXITCODE)" }
Write-Host "Flash OK"
