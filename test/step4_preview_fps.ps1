# Step 4: live preview window + real fps measurement
param(
    [string]$VideoName,
    [int]$W = 1280,
    [int]$H = 720
)

. "$PSScriptRoot\_common.ps1"
Assert-Ffmpeg
$VideoName = Resolve-VideoDevice -Name $VideoName

Write-Host "1) Opening preview window (1280x720)..." -ForegroundColor Cyan
Write-Host "   Watch for: smooth motion, correct exposure. Press Q or close window to continue."
& ffplay -f dshow -vcodec mjpeg -video_size "$($W)x$($H)" -i "video=$VideoName" -window_title "JQ-CAM12 preview (close to continue)" 2>&1 | Out-Null

Write-Host "2) Measuring actual fps over 150 frames (~5s)..." -ForegroundColor Cyan
$raw = & ffmpeg -hide_banner -f dshow -vcodec mjpeg -video_size "$($W)x$($H)" -i "video=$VideoName" -frames:v 150 -f null - 2>&1 | Out-String
if ($raw -match 'fps=\s*([\d.]+)') {
    $fps = [double]$Matches[1]
    Write-Host ("   Measured: ~{0:N1} fps" -f $fps) -ForegroundColor Green
    Write-Host "=== Judgement ===" -ForegroundColor Yellow
    if ($fps -ge 20) { Write-Host "   PASS (>= 20fps)" }
    else { Write-Host "   WEAK: below 20fps - try another USB port (avoid hubs), shorter cable" }
} else {
    Write-Host "[ERROR] could not measure fps. Output tail:" -ForegroundColor Red
    ($raw -split "`r?`n") | Where-Object { $_.Trim() } | Select-Object -Last 5 | Write-Host
}
