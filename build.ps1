<#
.SYNOPSIS
  Builds BetterReplay: native core tests, the WASM module, the toolbar panel,
  and a Community-ready package folder in dist\Community\betterreplay-recorder.

.EXAMPLE
  .\build.ps1                 # tests + module + panel + package folder
  .\build.ps1 -Install        # ...and copy it into the MSFS 2024 Community folder
#>
[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
$Root = $PSScriptRoot
$BuildDir = Join-Path $Root 'build'

function Write-Step([string]$Message) {
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Find-MSBuild {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { throw 'vswhere.exe not found; install Visual Studio 2022 or Build Tools 2022.' }
    $path = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
    if (-not $path) { throw 'MSBuild.exe not found.' }
    return $path
}

function Invoke-Native([string]$Exe, [string[]]$Arguments) {
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$([IO.Path]::GetFileName($Exe)) failed with exit code $LASTEXITCODE" }
}

function Invoke-Tests([string]$MSBuild) {
    Write-Step "Core tests ($Configuration|x64)"
    Invoke-Native $MSBuild @((Join-Path $Root 'tests\CoreTests.vcxproj'), '/nologo', '/v:minimal', '/m',
        "/p:Configuration=$Configuration", '/p:Platform=x64')
    Invoke-Native (Join-Path $BuildDir "tests\$Configuration\CoreTests.exe") @()
}

$msbuild = Find-MSBuild
if (-not $SkipTests) { Invoke-Tests $msbuild }
Write-Step 'Done'
