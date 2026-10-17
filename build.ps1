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
$ModuleName = 'BetterReplay.wasm'

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

function Get-Sdk {
    $sdk = $env:MSFS2024_SDK
    if (-not $sdk) { $sdk = $env:MSFS_SDK }
    if (-not $sdk -or -not (Test-Path (Join-Path $sdk 'WASM\llvm\bin\clang-cl.exe'))) {
        throw 'MSFS 2024 SDK not found: set MSFS2024_SDK (e.g. C:\MSFS 2024 SDK\).'
    }
    return $sdk.TrimEnd('\')
}

# Output goes straight to the console so it never becomes part of a function's return value.
function Invoke-Native([string]$Exe, [string[]]$Arguments) {
    & $Exe @Arguments | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$([IO.Path]::GetFileName($Exe)) failed with exit code $LASTEXITCODE" }
}

function Invoke-Tests([string]$MSBuild) {
    Write-Step "Core tests ($Configuration|x64)"
    Invoke-Native $MSBuild @((Join-Path $Root 'tests\CoreTests.vcxproj'), '/nologo', '/v:minimal', '/m',
        "/p:Configuration=$Configuration", '/p:Platform=x64')
    Invoke-Native (Join-Path $BuildDir "tests\$Configuration\CoreTests.exe") @()
}

# The MSFS2024 platform toolset registers an "MSFS" platform with MSBuild's VC targets.
function Test-MsfsToolset([string]$MSBuild) {
    $msbuildRoot = Split-Path (Split-Path (Split-Path $MSBuild))
    return Test-Path (Join-Path $msbuildRoot 'Microsoft\VC\v170\Platforms\MSFS')
}

function Build-ModuleWithToolset([string]$MSBuild, [string]$OutFile) {
    Write-Step "WASM module ($Configuration|MSFS, MSFS2024 platform toolset)"
    Invoke-Native $MSBuild @((Join-Path $Root 'src\module\BetterReplayModule.vcxproj'), '/nologo', '/v:minimal', '/m',
        "/p:Configuration=$Configuration", '/p:Platform=MSFS')
    if (-not (Test-Path $OutFile)) { throw "toolset build did not produce $OutFile" }
}

# Same ABI as the toolset output: links MSFS_WasmVersions.a and exports the version and
# allocator symbols the sim looks for (compared against a working MSFS 2024 module).
function Build-ModuleWithClang([string]$OutFile) {
    Write-Step "WASM module ($Configuration, SDK clang; MSFS2024 VS toolset not installed)"
    $sdk = Get-Sdk
    $clang = Join-Path $sdk 'WASM\llvm\bin\clang-cl.exe'
    $ld = Join-Path $sdk 'WASM\llvm\bin\wasm-ld.exe'
    $lib = Join-Path $sdk 'WASM\wasi-sysroot\lib\wasm32-wasi'
    $objDir = Join-Path $BuildDir "obj\module\$Configuration"
    New-Item -ItemType Directory -Force $objDir | Out-Null

    $cflags = @('--driver-mode=gcc', '--target=wasm32-unknown-wasi', "--sysroot=$sdk\WASM\wasi-sysroot",
        '-std=c++17', '-fno-exceptions', '-fms-extensions', '-fvisibility=hidden', '-ffunction-sections',
        '-fdata-sections', '-fno-stack-protector', '-fstack-size-section', '-mbulk-memory',
        '-Wall', '-Wextra', '-Werror=return-type',
        '-D_MSFS_WASM=1', '-D__wasi__', '-D_STRING_H_CPLUSPLUS_98_CONFORMANCE_',
        '-D_WCHAR_H_CPLUSPLUS_98_CONFORMANCE_', '-D_LIBCPP_HAS_NO_THREADS', '-D_WINDLL',
        "-I$sdk\WASM\include", "-I$sdk\SimConnect SDK\include", "-I$Root\src\core")
    if ($Configuration -eq 'Release') { $cflags += @('-O3', '-DNDEBUG') } else { $cflags += @('-O0', '-g', '-D_DEBUG') }

    $sources = @(Get-ChildItem (Join-Path $Root 'src\core\*.cpp')) + @(Get-ChildItem (Join-Path $Root 'src\module\*.cpp'))
    $objects = @()
    foreach ($src in $sources) {
        $obj = Join-Path $objDir ($src.BaseName + '.o')
        Invoke-Native $clang ($cflags + @('-c', $src.FullName, '-o', $obj))
        $objects += $obj
    }

    $exports = @('malloc', 'free', 'mallinfo', 'mchunkit_begin', 'mchunkit_next', 'get_pages_state',
        'mark_decommit_pages', '__wasm_call_ctors', 'GetSimConnectVersion', 'GetExtensionVersion',
        'GetExtensionVersionBuffer') | ForEach-Object { "--export=$_" }
    $ldflags = @('--no-entry', '--allow-undefined', '--export-dynamic', '--export-table', '--stack-guard-page',
        '-z', 'stack-size=1048576', '--gc-sections', "-L$lib") + $exports
    if ($Configuration -eq 'Release') { $ldflags += @('-O2', '--strip-debug') }
    $ldflags += $objects + @((Join-Path $sdk 'WASM\WasmVersions\MSFS_WasmVersions.a'), '-lc++', '-lc++abi', '-lc',
        (Join-Path $lib 'libclang_rt.builtins-wasm32.a'), '-o', $OutFile)
    Invoke-Native $ld $ldflags
}

function Build-Module([string]$MSBuild) {
    $outDir = Join-Path $BuildDir "module\$Configuration"
    New-Item -ItemType Directory -Force $outDir | Out-Null
    $outFile = Join-Path $outDir $ModuleName
    if (Test-MsfsToolset $MSBuild) {
        Build-ModuleWithToolset $MSBuild $outFile
    } else {
        Build-ModuleWithClang $outFile
    }
    Invoke-Native 'node' @((Join-Path $Root 'tools\check-wasm.mjs'), $outFile)
    return $outFile
}

$msbuild = Find-MSBuild
if (-not $SkipTests) { Invoke-Tests $msbuild }
$wasm = Build-Module $msbuild
Write-Step 'Done'
