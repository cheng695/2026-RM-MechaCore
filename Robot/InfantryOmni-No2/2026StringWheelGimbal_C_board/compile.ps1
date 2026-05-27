param(
    [ValidateSet("Debug", "Release")]
    [string]$BuildType = "Debug"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = $PSScriptRoot
$BuildDir = Join-Path $ProjectRoot "build\$BuildType"

Write-Host "=== 2026StringWheelGimbal_C_board Build ===" -ForegroundColor Cyan
Write-Host "Build type: $BuildType" -ForegroundColor Cyan
Write-Host ""

Write-Host "[1/2] Configuring CMake..." -ForegroundColor Yellow
& cmake --preset $BuildType -S $ProjectRoot
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake configure failed!" -ForegroundColor Red
    exit 1
}
Write-Host "Configure done." -ForegroundColor Green
Write-Host ""

Write-Host "[2/2] Building..." -ForegroundColor Yellow
& cmake --build --preset $BuildType
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}
Write-Host "Build succeeded." -ForegroundColor Green
Write-Host ""

$elf = Join-Path $BuildDir "$(Split-Path $ProjectRoot -Leaf).elf"
$hex = Join-Path $BuildDir "$(Split-Path $ProjectRoot -Leaf).hex"
$bin = Join-Path $BuildDir "$(Split-Path $ProjectRoot -Leaf).bin"
Write-Host "Output:" -ForegroundColor Cyan
Write-Host "  ELF: $elf"
Write-Host "  HEX: $hex"
Write-Host "  BIN: $bin"
