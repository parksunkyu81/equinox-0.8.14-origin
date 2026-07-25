[CmdletBinding()]
param(
  [switch]$SkipImageBuild,

  [switch]$VerifyOnly,

  [switch]$ListOnly,

  [switch]$Force
)

$ErrorActionPreference = "Stop"

$packageDir = $PSScriptRoot
$dockerfile = Join-Path $packageDir "Dockerfile.dfu"
$image = "comma-pedal-dfu:0.8.14-gm-lowimp"
$application = Join-Path $packageDir "pedal.bin.signed"
$bootstub = Join-Path $packageDir "bootstub.pedal.bin"
$expectedApplicationHash = "685DD63784B4B8C5F930286082AB6C75B231CF3C300A63CE96B214B409679578"
$expectedBootstubHash = "6F93267F4488C2C47333640E6F7AEC64FBEE78F0BEBD1BEA0180FD7AACEE802E"

foreach ($artifact in @($application, $bootstub)) {
  if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
    throw "Firmware artifact not found: $artifact"
  }
}

$applicationHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $application).Hash
$bootstubHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bootstub).Hash
if ($applicationHash -ne $expectedApplicationHash) {
  throw "Application SHA-256 mismatch. Expected $expectedApplicationHash, got $applicationHash"
}
if ($bootstubHash -ne $expectedBootstubHash) {
  throw "Bootstub SHA-256 mismatch. Expected $expectedBootstubHash, got $bootstubHash"
}

$signedData = [System.IO.File]::ReadAllBytes($application)
$declaredLength = [BitConverter]::ToUInt32($signedData, 0)
$markerOffset = $signedData.Length - 136
$marker = [Text.Encoding]::ASCII.GetString($signedData, $markerOffset, 4)
$version = [BitConverter]::ToUInt32($signedData, $markerOffset + 4)
if ($marker -ne "VERS" -or $version -ne 2 -or $declaredLength -ne ($signedData.Length - 128)) {
  throw "The signed pedal application structure is invalid."
}

docker info *> $null
if ($LASTEXITCODE -ne 0) {
  throw "Docker Desktop is not running."
}

if (-not $SkipImageBuild) {
  Write-Host "Building isolated STM32 DFU image..."
  docker build --file $dockerfile --tag $image $packageDir
  if ($LASTEXITCODE -ne 0) {
    throw "Docker DFU image build failed."
  }
}

$firmwareMount = "type=bind,source=$packageDir,target=/firmware,readonly"

if ($VerifyOnly) {
  docker run --rm --mount $firmwareMount $image verify
  exit $LASTEXITCODE
}

$usbArgs = @(
  "--privileged",
  "--mount", "type=bind,source=/dev/bus/usb,target=/dev/bus/usb"
)

if ($ListOnly) {
  & docker run --rm @usbArgs --mount $firmwareMount $image list
  exit $LASTEXITCODE
}

Write-Host "Application: $application"
Write-Host "SHA-256   : $applicationHash"
Write-Host "Bootstub  : $bootstub"
Write-Host "SHA-256   : $bootstubHash"
Write-Warning "This directly erases and rewrites the pedal STM32 flash."
Write-Warning "Disconnect the pedal completely from the vehicle. Use only its USB/DFU connection."

if (-not $Force) {
  $answer = Read-Host "Type DFU FLASH to continue"
  if ($answer -cne "DFU FLASH") {
    Write-Host "Cancelled."
    exit 0
  }
}

Write-Host "Starting direct USB DFU flash; Panda and CAN are not used..."
& docker run --rm @usbArgs `
  --mount $firmwareMount `
  --env "CONFIRM_PEDAL_DFU_FLASH=YES" `
  $image flash
$flashExitCode = $LASTEXITCODE

if ($flashExitCode -ne 0) {
  Write-Warning "Direct DFU flashing did not complete."
  Write-Host "Confirm that usbipd reports the STM32 device as Attached and that Docker can see /dev/bus/usb."
  exit $flashExitCode
}

Write-Host "Direct pedal USB DFU flash completed."
