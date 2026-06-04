# Flash build/AR22_LCU_v1_0F.hex to the target via ST-Link (STM32CubeProgrammer CLI).
# If several ST-Link probes are connected, auto-selects the one that actually
# reaches an STM32 target. Set $env:STLINK_SN to force a specific probe.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot\..").Path

$cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
if (-not (Test-Path $cli)) {
  $found = Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue
  if ($found) { $cli = $found.Source } else { throw "STM32_Programmer_CLI not found" }
}

$hex = "$root\build\AR22_LCU_v1_0F.hex"
if (-not (Test-Path $hex)) { throw "$hex not found - build first" }

$targetSN = $null
if ($env:STLINK_SN) {
  $targetSN = $env:STLINK_SN
  Write-Host "Using ST-Link SN from STLINK_SN: $targetSN"
} else {
  # NOTE: do not use '2>&1' on the native exe in Windows PowerShell 5.1 (it mangles stdout).
  $listOut = (& $cli --list | Out-String)
  $sns = @()
  foreach ($m in [regex]::Matches($listOut, 'ST-LINK SN\s*:\s*(\S+)')) { $sns += $m.Groups[1].Value }
  $sns = $sns | Select-Object -Unique
  if ($sns.Count -eq 0) { throw "No ST-Link probe detected. Connect the probe wired to the board's SWD." }

  foreach ($sn in $sns) {
    Write-Host "Checking probe $sn for an STM32 target..."
    & $cli -c port=SWD mode=Hotplug sn=$sn | Out-Null
    if ($LASTEXITCODE -eq 0) { $targetSN = $sn; break }
  }
  if (-not $targetSN) {
    throw ("Connected probe(s) [{0}] cannot reach an STM32 target via SWD. " -f ($sns -join ', ')) +
          "Connect the ST-Link that is wired to the board's SWD, or set `$env:STLINK_SN."
  }
  Write-Host "Selected ST-Link SN: $targetSN"
}

& $cli -c port=SWD freq=4000 sn=$targetSN -d $hex -v -rst
if ($LASTEXITCODE -ne 0) { throw "Flash failed (CLI exit $LASTEXITCODE)" }
Write-Host "Flash OK"
