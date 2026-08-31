# Shared helper for camera test scripts (requires ffmpeg on PATH)

function Assert-Ffmpeg {
    if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) {
        Write-Host "[ERROR] ffmpeg not found. Install it first:" -ForegroundColor Red
        Write-Host "    winget install --id Gyan.FFmpeg -e"
        Write-Host "    (then reopen the terminal)"
        exit 1
    }
}

function Get-DshowDevices {
    # Returns an object with Video[] and Audio[] DirectShow device names
    $raw = & ffmpeg -hide_banner -list_devices true -f dshow -i dummy 2>&1 | Out-String
    $video = @()
    $audio = @()
    foreach ($line in ($raw -split "`r?`n")) {
        if ($line -match '"([^"]+)"\s*\((video)') { $video += $Matches[1] }
        elseif ($line -match '"([^"]+)"\s*\((audio)') { $audio += $Matches[1] }
    }
    [pscustomobject]@{ Video = $video; Audio = $audio }
}

function Resolve-VideoDevice {
    param([string]$Name)
    if ($Name) { return $Name }
    $d = Get-DshowDevices
    if ($d.Video.Count -gt 0) {
        Write-Host ("Using video device: {0}" -f $d.Video[0]) -ForegroundColor Yellow
        return $d.Video[0]
    }
    Write-Host "[ERROR] No DirectShow video device found." -ForegroundColor Red
    Write-Host "  - Is the camera plugged in? All 4 wires connected (5V/GND/D+/D-)?"
    Write-Host "  - Run step1_check_devices.ps1 for details."
    exit 1
}

function New-OutputDir {
    $dir = Join-Path $PSScriptRoot "output"
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    return $dir
}
