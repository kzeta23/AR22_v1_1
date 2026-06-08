# Generate compile_commands.json for clangd / C IntelliSense (matches build.ps1 flags).
# Run after adding/removing source files. Output: <project root>/compile_commands.json
$ErrorActionPreference = 'Stop'
$root  = (Resolve-Path "$PSScriptRoot\..").Path
$rootF = $root -replace '\\','/'

$gcc = "C:/Program Files (x86)/Arm/GNU Toolchain mingw-w64-i686-arm-none-eabi/bin/arm-none-eabi-gcc.exe"

# --- Source files (same set as the build) ---
$srcs = @()
$srcs += Get-ChildItem "$root\Core\Src" -Filter *.c | ForEach-Object FullName
$srcs += Get-ChildItem "$root\Drivers\STM32F4xx_HAL_Driver\Src" -Filter *.c | ForEach-Object FullName
$srcs += Get-ChildItem "$root\MyLib" -Recurse -Filter *.c | ForEach-Object FullName

# --- Include dirs (absolute, forward-slash) ---
$incRel = @(
  "Core/Inc",
  "Drivers/STM32F4xx_HAL_Driver/Inc",
  "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
  "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
  "Drivers/CMSIS/Include",
  "MyLib",
  "MyLib/SSD1322_OLED_lib_NB",
  "MyLib/SSD1322_OLED_lib_NB/Fonts",
  "MyLib/SSD1322_OLED_lib_NB/Bitmap",
  "MyLib/MOVING_AVERAGE",
  "MyLib/EEPROM_I2C"
)
$inc = $incRel | ForEach-Object { "-I$rootF/$_" }

$flags = @(
  "-mcpu=cortex-m4","-mfpu=fpv4-sp-d16","-mfloat-abi=hard","-mthumb",
  "-DUSE_HAL_DRIVER","-DSTM32F446xx","-DDEBUG","-std=gnu11"
) + $inc

$entries = foreach ($s in $srcs) {
  $sf = $s -replace '\\','/'
  [pscustomobject]@{
    directory = $rootF
    file      = $sf
    arguments = @($gcc) + $flags + @("-c", $sf)
  }
}

# Force a JSON array even if only one entry, write UTF-8 without BOM (clangd-friendly)
$json = ConvertTo-Json @($entries) -Depth 6
[System.IO.File]::WriteAllText("$root\compile_commands.json", $json)
Write-Host "Wrote compile_commands.json with $($entries.Count) entries"
