param(
    [ValidateSet("Debug", "Release")]
    [string]$BuildType = "Debug",

    [ValidateSet("openocd", "jlink", "stm32cubeprogrammer")]
    [string]$Programmer = "openocd",

    [string]$Interface = "cmsis-dap"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = $PSScriptRoot
$ProjectName = Split-Path $ProjectRoot -Leaf
$BuildDir = Join-Path $ProjectRoot "build\$BuildType"
$FirmwareFile = Join-Path $BuildDir "$ProjectName.elf"

if (-not (Test-Path $FirmwareFile)) {
    Write-Host "Firmware not found: $FirmwareFile" -ForegroundColor Red
    Write-Host "Please run compile.ps1 first." -ForegroundColor Red
    exit 1
}

Write-Host "=== Download to STM32F407 ===" -ForegroundColor Cyan
Write-Host "Programmer: $Programmer" -ForegroundColor Cyan
Write-Host "Firmware: $FirmwareFile" -ForegroundColor Cyan
Write-Host ""

function Download-OpenOCD {
    $openocdCfg = Join-Path $ProjectRoot "openocd.cfg"
    if (-not (Test-Path $openocdCfg)) {
        Write-Host "openocd.cfg not found, using default config..." -ForegroundColor Yellow
        $cfgContent = @"
source [find interface/$Interface.cfg]
source [find target/stm32f4x.cfg]
init
reset halt
program $FirmwareFile verify reset exit
"@
        $cfgContent | Set-Content -Path $openocdCfg -Encoding UTF8
    }

    Write-Host "Running OpenOCD..." -ForegroundColor Yellow
    & openocd -f $openocdCfg
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Download failed!" -ForegroundColor Red
        exit 1
    }
    Write-Host "Download succeeded!" -ForegroundColor Green
}

function Download-JLink {
    $jlinkScript = Join-Path $ProjectRoot "download.jlink"
    $hexFile = Join-Path $BuildDir "$ProjectName.hex"
    $scriptContent = @"
device STM32F407IG
si 1
speed 8000
loadfile $hexFile
r
g
exit
"@
    $scriptContent | Set-Content -Path $jlinkScript -Encoding ASCII

    Write-Host "Running JLink..." -ForegroundColor Yellow
    & JLink -device STM32F407IG -if SWD -speed 8000 -autoconnect 1 -CommanderScript $jlinkScript
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Download failed!" -ForegroundColor Red
        exit 1
    }
    Write-Host "Download succeeded!" -ForegroundColor Green
}

function Download-STM32CubeProgrammer {
    $binFile = Join-Path $BuildDir "$ProjectName.bin"
    Write-Host "Running STM32CubeProgrammer..." -ForegroundColor Yellow
    & STM32_Programmer_CLI -c port=SWD -w $binFile 0x08000000 -v -rst
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Download failed!" -ForegroundColor Red
        exit 1
    }
    Write-Host "Download succeeded!" -ForegroundColor Green
}

switch ($Programmer) {
    "openocd" { Download-OpenOCD }
    "jlink" { Download-JLink }
    "stm32cubeprogrammer" { Download-STM32CubeProgrammer }
}
