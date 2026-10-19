<#
.SYNOPSIS
  Builds BetterReplay: native core tests, the WASM module, the toolbar panel, and a
  Community-ready package folder in dist\Community\betterreplay-recorder.

.DESCRIPTION
  The toolbar panel definition must be compiled to an .spb by the sim's package builder.
  Run once with -Package (close the sim first: fspackagetool starts FlightSimulator2024.exe in
  package-builder mode). The result is cached in package\spb\ and reused by later builds until
  package\PackageSources\InGamePanels\InGamePanel_BetterReplay.xml changes.

  The version comes from package\PackageDefinitions\betterreplay-recorder.xml; the module's
  kVersion in src\module\Module.cpp must match it for -Zip.

.EXAMPLE
  .\build.ps1 -Package          # first build: also compiles the panel .spb
  .\build.ps1                   # tests + module + panel + package folder
  .\build.ps1 -Install          # ...and copy it into the MSFS 2024 Community folder
  .\build.ps1 -Zip              # ...and zip it for a GitHub release
#>
[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [switch]$SkipTests,
    [switch]$Package,
    [switch]$Install,
    [string]$CommunityPath = '',
    [switch]$Zip
)

$ErrorActionPreference = 'Stop'
$Root = $PSScriptRoot
$BuildDir = Join-Path $Root 'build'
$DistDir = Join-Path $Root 'dist\Community'
$ModuleName = 'BetterReplay.wasm'
$PackageName = 'betterreplay-recorder'
$PackageDefinition = Join-Path $Root "package\PackageDefinitions\$PackageName.xml"
$PackageVersion = ([xml](Get-Content $PackageDefinition -Raw)).AssetPackage.Version
$PanelDirName = 'BetterReplayPanel'
$SpbName = 'InGamePanel_BetterReplay.spb'
$SpbSource = Join-Path $Root 'package\PackageSources\InGamePanels\InGamePanel_BetterReplay.xml'
$SpbCacheDir = Join-Path $Root 'package\spb'
$ProfilesDir = Join-Path $Root 'profiles'

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

function Reset-Directory([string]$Path) {
    if (Test-Path $Path) { Remove-Item -Recurse -Force $Path }
    New-Item -ItemType Directory -Force $Path | Out-Null
}

# ------------------------------------------------------------------------------------- tests

function Invoke-Tests([string]$MSBuild) {
    Write-Step "Core tests ($Configuration|x64)"
    Invoke-Native $MSBuild @((Join-Path $Root 'tests\CoreTests.vcxproj'), '/nologo', '/v:minimal', '/m',
        "/p:Configuration=$Configuration", '/p:Platform=x64')
    Invoke-Native (Join-Path $BuildDir "tests\$Configuration\CoreTests.exe") @()
}

# ------------------------------------------------------------------------------------ module

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

# ------------------------------------------------------------------------------------- panel

function Build-Panel {
    Write-Step 'Toolbar panel (TypeScript -> JavaScript)'
    $out = Join-Path $BuildDir 'panel\BetterReplayPanel.js'
    Invoke-Native 'node' @('--no-warnings', (Join-Path $Root 'tools\strip-types.mjs'),
        (Join-Path $Root 'src\panel\BetterReplayPanel.ts'), $out)
    return $out
}

# ----------------------------------------------------------------------------------- package

# build\package mirrors a DevMode project: BetterReplay.xml + PackageDefinitions + PackageSources.
function New-PackageStage([string]$Wasm, [string]$PanelJs) {
    $stage = Join-Path $BuildDir 'package'
    Reset-Directory $stage
    Copy-Item (Join-Path $Root 'package\BetterReplay.xml') $stage
    Copy-Item -Recurse (Join-Path $Root 'package\PackageDefinitions') $stage
    Copy-Item -Recurse (Join-Path $Root 'package\PackageSources') $stage
    $panelDir = Join-Path $stage "PackageSources\html_ui\InGamePanels\$PanelDirName"
    New-Item -ItemType Directory -Force $panelDir | Out-Null
    Copy-Item (Join-Path $Root 'src\panel\BetterReplayPanel.html'), (Join-Path $Root 'src\panel\BetterReplayPanel.css'), $PanelJs $panelDir
    $modulesDir = Join-Path $stage 'PackageSources\modules'
    New-Item -ItemType Directory -Force $modulesDir | Out-Null
    Copy-Item $Wasm $modulesDir
    return $stage
}

# fspackagetool asks Steam to start FlightSimulator2024.exe in package-builder mode. Steam
# refuses ("game is already running") if the sim is open, or if a previous instance exited
# abnormally and Steam never cleared its running flag.
function Assert-PackageToolCanRun {
    if (Get-Process -Name 'FlightSimulator2024' -ErrorAction SilentlyContinue) {
        throw 'Close Microsoft Flight Simulator 2024 first: fspackagetool starts its own copy in package-builder mode.'
    }
    $steamApp = Get-ItemProperty 'HKCU:\Software\Valve\Steam\Apps\2537590' -ErrorAction SilentlyContinue
    if ($steamApp -and $steamApp.Running -eq 1) {
        throw ('Steam still thinks MSFS 2024 is running, but it is not (an earlier instance ended abnormally), ' +
            'so it would refuse to start the package builder. In Steam, click Stop on Microsoft Flight Simulator 2024, ' +
            'or exit Steam completely and start it again, then re-run this build.')
    }
}

# Hashes the text with normalized line endings so a CRLF checkout still matches the cache.
function Get-SpbSourceHash {
    $text = [IO.File]::ReadAllText($SpbSource).Replace("`r`n", "`n")
    $sha = [Security.Cryptography.SHA256]::Create()
    $bytes = $sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($text))
    return [BitConverter]::ToString($bytes).Replace('-', '')
}

function Invoke-PackageTool([string]$Stage) {
    Write-Step 'Compiling the panel .spb with fspackagetool (starts FlightSimulator2024.exe in package-builder mode)'
    $sdk = Get-Sdk
    $tool = Join-Path $sdk 'Tools\bin\fspackagetool.exe'
    $project = Join-Path $Stage 'BetterReplay.xml'
    Invoke-Native $tool @($project, '-nopause', '-rebuild')
    $built = Join-Path $Stage "Packages\$PackageName"
    $spb = Join-Path $built "InGamePanels\$SpbName"
    if (-not (Test-Path $spb)) {
        throw "fspackagetool did not produce $spb. See %APPDATA%\Microsoft Flight Simulator 2024\BuilderLogError.txt"
    }
    New-Item -ItemType Directory -Force $SpbCacheDir | Out-Null
    Copy-Item $spb $SpbCacheDir -Force
    Set-Content -Encoding ascii (Join-Path $SpbCacheDir 'source.sha256') (Get-SpbSourceHash)
    # Keep the tool's manifest: it carries the compatibility version the sim expects.
    Copy-Item (Join-Path $built 'manifest.json') (Join-Path $SpbCacheDir 'manifest.tool.json') -Force
    Write-Host "  cached $SpbName in package\spb\"
}

function Get-CachedSpb {
    $spb = Join-Path $SpbCacheDir $SpbName
    $hashFile = Join-Path $SpbCacheDir 'source.sha256'
    if (-not (Test-Path $spb) -or -not (Test-Path $hashFile)) { return $null }
    if ((Get-Content $hashFile -Raw).Trim() -ne (Get-SpbSourceHash)) {
        Write-Warning 'InGamePanel_BetterReplay.xml changed since the cached .spb was built; run .\build.ps1 -Package'
        return $null
    }
    return $spb
}

function Write-Manifest([string]$Dir, [long]$TotalSize) {
    $compat = $null
    $toolManifest = Join-Path $SpbCacheDir 'manifest.tool.json'
    if (Test-Path $toolManifest) { $compat = (Get-Content $toolManifest -Raw | ConvertFrom-Json) }
    $manifest = [ordered]@{
        dependencies = @()
        content_type = 'MISC'
        title = 'BetterReplay'
        manufacturer = ''
        creator = 'BetterReplay'
        package_version = $PackageVersion
        minimum_game_version = '1.0.0'
        release_notes = [ordered]@{ neutral = [ordered]@{ LastUpdate = ''; OlderHistory = '' } }
        total_package_size = "$TotalSize"
        package_order_hint = 'MISC'
        export_type = 'Community'
        builder = 'Microsoft Flight Simulator 2024'
    }
    if ($compat -and $compat.minimum_game_version) { $manifest.minimum_game_version = $compat.minimum_game_version }
    if ($compat -and $compat.minimum_compatibility_version) {
        $manifest.minimum_compatibility_version = $compat.minimum_compatibility_version
    }
    $json = $manifest | ConvertTo-Json -Depth 5
    [IO.File]::WriteAllText((Join-Path $Dir 'manifest.json'), $json, (New-Object Text.UTF8Encoding $false))
}

# layout.json lists every file with its size and FILETIME, lowercased and sorted, like the
# package builder writes it.
function Write-Layout([string]$Dir) {
    $prefix = (Resolve-Path $Dir).Path.TrimEnd('\') + '\'
    $entries = Get-ChildItem -Recurse -File $Dir |
        Where-Object { $_.Name -ne 'layout.json' -and $_.Name -ne 'manifest.json' } |
        ForEach-Object {
            [ordered]@{
                path = $_.FullName.Substring($prefix.Length).Replace('\', '/').ToLowerInvariant()
                size = $_.Length
                date = $_.LastWriteTimeUtc.ToFileTimeUtc()
            }
        } | Sort-Object { $_.path }
    $total = 0
    foreach ($e in $entries) { $total += $e.size }
    $json = [ordered]@{ content = @($entries) } | ConvertTo-Json -Depth 5
    [IO.File]::WriteAllText((Join-Path $Dir 'layout.json'), $json, (New-Object Text.UTF8Encoding $false))
    return $total
}

function New-CommunityPackage([string]$Stage) {
    Write-Step "Package folder dist\Community\$PackageName"
    $dir = Join-Path $DistDir $PackageName
    Reset-Directory $dir
    Copy-Item -Recurse (Join-Path $Stage 'PackageSources\html_ui') $dir
    Copy-Item -Recurse (Join-Path $Stage 'PackageSources\modules') $dir
    $spb = Get-CachedSpb
    if ($spb) {
        $panels = Join-Path $dir 'InGamePanels'
        New-Item -ItemType Directory -Force $panels | Out-Null
        Copy-Item $spb $panels
    } else {
        Write-Warning "No compiled panel ($SpbName): the recorder will run but the toolbar panel won't appear. Run .\build.ps1 -Package once (sim closed)."
    }
    # The included control profiles (profiles\*.json, exported with tools\export-profiles.mjs), in one
    # file as the module reads them.
    if (Get-ChildItem (Join-Path $ProfilesDir '*.json') -ErrorAction SilentlyContinue) {
        Invoke-Native 'node' @((Join-Path $Root 'tools\pack-profiles.mjs'), $ProfilesDir,
            (Join-Path $dir 'profiles\aircraft.json'))
    }
    $total = Write-Layout $dir
    Write-Manifest $dir $total
    Write-Host "  $dir ($([math]::Round($total / 1KB)) KB)"
    return $dir
}

function Find-CommunityFolder {
    if ($CommunityPath) { return $CommunityPath }
    $candidates = @(
        (Join-Path $env:APPDATA 'Microsoft Flight Simulator 2024\UserCfg.opt'),
        (Join-Path $env:LOCALAPPDATA 'Packages\Microsoft.Limitless_8wekyb3d8bbwe\LocalCache\UserCfg.opt')
    )
    foreach ($cfg in $candidates) {
        if (-not (Test-Path $cfg)) { continue }
        $line = Select-String -Path $cfg -Pattern '^\s*InstalledPackagesPath\s+"(.+)"' | Select-Object -First 1
        if ($line) { return (Join-Path $line.Matches[0].Groups[1].Value 'Community') }
    }
    throw 'Could not find the MSFS 2024 Community folder; pass -CommunityPath.'
}

function Install-Package([string]$PackageDir) {
    $community = Find-CommunityFolder
    $target = Join-Path $community $PackageName
    Write-Step "Installing to $target"
    if (Test-Path $target) { Remove-Item -Recurse -Force $target }
    Copy-Item -Recurse $PackageDir $target
}

# ----------------------------------------------------------------------------------- release

# Checked before building, so a release zip can't miss its panel or carry two versions.
function Assert-ReleaseReady {
    if ($Configuration -ne 'Release' -or $SkipTests) { throw '-Zip needs a Release build with the tests run.' }
    if ($PackageVersion -notmatch '^\d+\.\d+\.\d+$') {
        throw "Version '$PackageVersion' in $PackageName.xml isn't major.minor.patch."
    }
    $moduleSource = Get-Content (Join-Path $Root 'src\module\Module.cpp') -Raw
    $moduleVersion = [regex]::Match($moduleSource, 'kVersion\s*=\s*"([^"]*)"').Groups[1].Value
    if ($moduleVersion -ne $PackageVersion) {
        throw ("Version mismatch: $PackageName.xml says $PackageVersion, kVersion in src\module\Module.cpp says " +
            "'$moduleVersion'. Set both to the release version.")
    }
    if (-not $Package -and -not (Get-CachedSpb)) {
        throw "No up-to-date compiled panel ($SpbName) for the release. Run .\build.ps1 -Package -Zip (sim closed)."
    }
}

# Git's output, or $null if the command fails (no upstream, no such tag...).
function Invoke-Git([string[]]$Arguments) {
    $ErrorActionPreference = 'Continue'
    $out = & git -C $Root @Arguments 2>$null
    if ($LASTEXITCODE -ne 0) { return $null }
    return $out
}

# The zip holds the package folder itself, with forward-slash entries, so it extracts straight
# into Community.
function New-ReleaseZip([string]$PackageDir) {
    $zip = Join-Path (Split-Path $DistDir) "$PackageName-$PackageVersion.zip"
    Write-Step "Release zip dist\$(Split-Path -Leaf $zip)"
    if (Test-Path $zip) { Remove-Item -Force $zip }
    Add-Type -AssemblyName System.IO.Compression, System.IO.Compression.FileSystem
    $prefix = (Resolve-Path $PackageDir).Path.TrimEnd('\') + '\'
    $archive = [IO.Compression.ZipFile]::Open($zip, [IO.Compression.ZipArchiveMode]::Create)
    try {
        foreach ($file in Get-ChildItem -Recurse -File $PackageDir) {
            $entry = "$PackageName/" + $file.FullName.Substring($prefix.Length).Replace('\', '/')
            [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive, $file.FullName, $entry,
                [IO.Compression.CompressionLevel]::Optimal) | Out-Null
        }
    } finally {
        $archive.Dispose()
    }
    Write-Host "  $zip ($([math]::Round((Get-Item $zip).Length / 1KB)) KB)"
}

# The release is published in the browser: this prints a link to GitHub's new-release form with
# the tag, the commit and the title filled in. Publishing creates the tag.
function Write-ReleaseLink {
    $tag = "v$PackageVersion"
    $commit = Invoke-Git @('rev-parse', 'HEAD')
    if (-not $commit) { Write-Warning 'Not a git checkout: tag the release by hand.'; return }
    if (Invoke-Git @('status', '--porcelain')) {
        Write-Warning "Uncommitted changes: the zip doesn't match commit $($commit.Substring(0, 7)). Commit, push and rebuild."
    }
    $ahead = Invoke-Git @('rev-list', '--count', '@{u}..HEAD')
    if ($ahead -and [int]$ahead -gt 0) {
        Write-Warning "$ahead commit(s) not pushed: push before publishing, the release tags commit $($commit.Substring(0, 7))."
    }
    $tagged = Invoke-Git @('rev-list', '-n', '1', $tag)
    if ($tagged -and $tagged -ne $commit) {
        Write-Warning "Tag $tag is already on commit $($tagged.Substring(0, 7)): bump the version for a new release."
    }
    $remote = Invoke-Git @('remote', 'get-url', 'origin')
    $match = [regex]::Match("$remote", 'github\.com[:/](.+?)(\.git)?$')
    if (-not $match.Success) { Write-Warning "origin isn't on GitHub: create release $tag by hand."; return }
    $title = [uri]::EscapeDataString("BetterReplay $PackageVersion")
    Write-Host "  Publish: open the link, add release notes, attach the zip, then Publish release (tags $tag on $($commit.Substring(0, 7))):"
    Write-Host "  https://github.com/$($match.Groups[1].Value)/releases/new?tag=$tag&target=$commit&title=$title"
}

# -------------------------------------------------------------------------------------- main

# Fail fast, before the slower build steps.
if ($Package) { Assert-PackageToolCanRun }
if ($Zip) { Assert-ReleaseReady }
if ($Install -and (Get-Process -Name 'FlightSimulator2024' -ErrorAction SilentlyContinue)) {
    throw ('Quit Microsoft Flight Simulator 2024 before installing: the module only loads at startup and its ' +
        'files are in use while the sim runs. (You can build without -Install while it is running.)')
}
$msbuild = Find-MSBuild
if (-not $SkipTests) { Invoke-Tests $msbuild }
$wasm = Build-Module $msbuild
$panelJs = Build-Panel
$stage = New-PackageStage $wasm $panelJs
if ($Package) { Invoke-PackageTool $stage }
$packageDir = New-CommunityPackage $stage
if ($Install) { Install-Package $packageDir }
if ($Zip) {
    New-ReleaseZip $packageDir
    Write-ReleaseLink
}
Write-Step 'Done'
