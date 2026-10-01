<#
.SYNOPSIS
    Build the FlatCAM EVO Windows application and its installer.

.DESCRIPTION
    Runs PyInstaller over packaging\flatcam.spec to produce dist\FlatCAM, then compiles
    packaging\flatcam.iss with Inno Setup to produce dist\FlatCAM_EVO_Setup.exe.

    Needs Python 3.12 or later with the packages from requirements.txt and pyinstaller installed,
    and Inno Setup 6 for the installer step.

.PARAMETER Python
    The Python interpreter to build with. Defaults to the one on PATH.

.PARAMETER Version
    The version shown by the installer. Defaults to the version in appMain.py.

.PARAMETER SkipInstaller
    Only produce the dist\FlatCAM folder, do not run Inno Setup.

.EXAMPLE
    .\packaging\build_win.ps1
    .\packaging\build_win.ps1 -Python .\.venv312\Scripts\python.exe -SkipInstaller
#>
[CmdletBinding()]
param(
    [string]$Python = 'python',
    [string]$Version,
    [switch]$SkipInstaller
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    Write-Host "Building FlatCAM EVO in $repoRoot" -ForegroundColor Cyan

    # ---------------------------------------------------------------- check the interpreter
    $pythonVersion = & $Python -c "import sys; print('%d.%d' % sys.version_info[:2])"
    if ($LASTEXITCODE -ne 0) {
        throw "Could not run the Python interpreter '$Python'."
    }
    Write-Host "Python $pythonVersion ($Python)"

    $major, $minor = $pythonVersion.Split('.')
    if ([int]$major -lt 3 -or ([int]$major -eq 3 -and [int]$minor -lt 12)) {
        throw "FlatCAM EVO needs Python 3.12 or later, found $pythonVersion."
    }

    & $Python -c "import PyInstaller" 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller is not installed for '$Python'. Run: $Python -m pip install pyinstaller"
    }

    # ---------------------------------------------------------------- the version to advertise
    if (-not $Version) {
        $Version = & $Python -c "import re, io; print(re.search(r'^\s+version = \""(.+)\""', io.open('appMain.py', encoding='utf-8').read(), re.M).group(1))"
        if ($LASTEXITCODE -ne 0 -or -not $Version) {
            $Version = 'Unstable'
        }
    }
    Write-Host "Version: $Version"

    # ---------------------------------------------------------------- freeze the application
    Write-Host "`nRunning PyInstaller ..." -ForegroundColor Cyan
    & $Python -m PyInstaller 'packaging\flatcam.spec' --noconfirm --distpath dist --workpath build
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller failed."
    }

    $appExe = Join-Path $repoRoot 'dist\FlatCAM\FlatCAM.exe'
    if (-not (Test-Path $appExe)) {
        throw "PyInstaller did not produce $appExe."
    }
    Write-Host "Built dist\FlatCAM\FlatCAM.exe" -ForegroundColor Green

    if ($SkipInstaller) {
        Write-Host "Skipping the installer." -ForegroundColor Yellow
        return
    }

    # ---------------------------------------------------------------- build the installer
    $iscc = Get-Command 'ISCC.exe' -ErrorAction SilentlyContinue
    if (-not $iscc) {
        foreach ($candidate in @(
            "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
            "$env:ProgramFiles\Inno Setup 6\ISCC.exe",
            # Inno Setup can also be installed for the current user only
            "$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe"
        )) {
            if (Test-Path $candidate) {
                $iscc = $candidate
                break
            }
        }
    } else {
        $iscc = $iscc.Source
    }

    if (-not $iscc) {
        Write-Warning "Inno Setup 6 was not found, therefore no Setup.exe was built. The application in dist\FlatCAM does run as it is. Install Inno Setup from https://jrsoftware.org/isdl.php and run this script again to get the installer."
        return
    }

    Write-Host "`nRunning Inno Setup ($iscc) ..." -ForegroundColor Cyan
    & $iscc "/DMyAppVersion=$Version" 'packaging\flatcam.iss'
    if ($LASTEXITCODE -ne 0) {
        throw "Inno Setup failed."
    }

    Write-Host "Built dist\FlatCAM_EVO_Setup.exe" -ForegroundColor Green
}
finally {
    Pop-Location
}
