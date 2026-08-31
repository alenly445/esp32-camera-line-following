# Step 5: record 5 seconds from the camera's microphone, then play it back
param(
    [string]$AudioName,
    [int]$Seconds = 5
)

. "$PSScriptRoot\_common.ps1"
Assert-Ffmpeg
$d = Get-DshowDevices
if (-not $AudioName) {
    if ($d.Audio.Count -eq 0) {
        Write-Host "[ERROR] No audio input device found." -ForegroundColor Red
        Write-Host "  The 3-in-1 board needs a mic capsule plugged into its 2P mic socket;"
        Write-Host "  without it, recording is silent but the CAMERA itself is still fine."
        exit 1
    }
    $AudioName = $d.Audio[0]
}

$outDir = New-OutputDir
$file = Join-Path $outDir "mic_test.wav"
Write-Host ("Recording {0}s from '{1}' ... SAY SOMETHING NOW" -f $Seconds, $AudioName) -ForegroundColor Cyan
& ffmpeg -hide_banner -f dshow -i "audio=$AudioName" -t $Seconds -y $file 2>&1 | Out-Null
if (Test-Path $file) {
    Write-Host "Playing back ..." -ForegroundColor Green
    (New-Object Media.SoundPlayer -ArgumentList $file).PlaySync()
    Write-Host "=== Judgement ===" -ForegroundColor Yellow
    Write-Host "  PASS = you hear your own voice (played through the DEFAULT speaker)"
} else {
    Write-Host "[ERROR] recording failed." -ForegroundColor Red
}
