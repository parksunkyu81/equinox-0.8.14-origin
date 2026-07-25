[CmdletBinding()]
param(
  [string]$OutputDirectory,

  [switch]$SkipImageBuild
)

$ErrorActionPreference = "Stop"

$packageDir = $PSScriptRoot
$workspace = (Resolve-Path -LiteralPath (Join-Path $packageDir "..\..")).Path
$dockerfile = Join-Path $packageDir "Dockerfile.build"
$containerBuildScript = "/source/output/pedal-firmware-gm-low-impedance-0.8.14/build_pedal_in_docker.sh"
$image = "comma-pedal-builder:0.8.14-gm-lowimp"

if (-not $OutputDirectory) {
  $stamp = Get-Date -Format "yyyyMMdd-HHmmss"
  $OutputDirectory = Join-Path $packageDir (Join-Path "compiled" $stamp)
} elseif (-not [System.IO.Path]::IsPathRooted($OutputDirectory)) {
  $OutputDirectory = Join-Path (Get-Location).Path $OutputDirectory
}

$outputItem = New-Item -ItemType Directory -Path $OutputDirectory -Force
$resolvedOutput = $outputItem.FullName

docker info *> $null
if ($LASTEXITCODE -ne 0) {
  throw "Docker Desktop is not running."
}

if (-not $SkipImageBuild) {
  Write-Host "Building isolated pedal compiler image..."
  docker build --file $dockerfile --tag $image $packageDir
  if ($LASTEXITCODE -ne 0) {
    throw "Docker compiler image build failed."
  }
}

$dockerArgs = @(
  "run",
  "--rm",
  "--mount", "type=bind,source=$workspace,target=/source,readonly",
  "--mount", "type=bind,source=$resolvedOutput,target=/out",
  $image,
  "bash",
  $containerBuildScript
)

Write-Host "Compiling pedal firmware on this PC only..."
& docker @dockerArgs
if ($LASTEXITCODE -ne 0) {
  throw "Pedal firmware compilation failed."
}

$signedFirmware = Join-Path $resolvedOutput "pedal.bin.signed"
$unsignedFirmware = Join-Path $resolvedOutput "pedal.bin"
$bootstub = Join-Path $resolvedOutput "bootstub.pedal.bin"

foreach ($artifact in @($signedFirmware, $unsignedFirmware, $bootstub)) {
  if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
    throw "Expected build artifact was not created: $artifact"
  }
}

$signedData = [System.IO.File]::ReadAllBytes($signedFirmware)
$declaredLength = [BitConverter]::ToUInt32($signedData, 0)
$markerOffset = $signedData.Length - 136
$marker = [Text.Encoding]::ASCII.GetString($signedData, $markerOffset, 4)
$version = [BitConverter]::ToUInt32($signedData, $markerOffset + 4)
if ($marker -ne "VERS" -or $version -ne 2 -or $declaredLength -ne ($signedData.Length - 128)) {
  throw "The generated signed firmware structure is invalid."
}

Write-Host ""
Write-Host "Compilation completed without accessing Panda or CAN hardware."
Write-Host "Output: $resolvedOutput"
Get-FileHash -Algorithm SHA256 -LiteralPath $signedFirmware, $unsignedFirmware, $bootstub |
  Format-Table -AutoSize

