# Step 2: list formats supported by the camera (resolution / fps / codec)
# The MJPEG 1280x720@30 line is exactly what the ESP32 UVC host will request later.
param([string]$VideoName)

. "$PSScriptRoot\_common.ps1"
Assert-Ffmpeg
$VideoName = Resolve-VideoDevice -Name $VideoName

Write-Host "--- Formats supported by '$VideoName' ---" -ForegroundColor Cyan
$raw = & ffmpeg -hide_banner -list_options true -f dshow -i "video=$VideoName" 2>&1 | Out-String
$fmt = ($raw -split "`r?`n") | Where-Object { $_ -match 'vcodec=' } | ForEach-Object {
    ($_ -replace '^\[dshow[^\]]*\]\s*', '').Trim()
}
if ($fmt) {
    $fmt | Write-Host
    Write-Host ""
    Write-Host "=== Judgement ===" -ForegroundColor Yellow
    Write-Host "  - Need a line like: vcodec=mjpeg min s=1280x720 fps=30"
    Write-Host "  - mjpeg = compressed stream (what ESP32-S3 wants; yuy2/raw is too heavy)"
    Write-Host "  - Write down the largest mjpeg resolution + fps for the ESP32 stage."
} else {
    Write-Host "[ERROR] could not read format list. Raw output:" -ForegroundColor Red
    ($raw -split "`r?`n") | Select-Object -Last 8 | Write-Host
}
