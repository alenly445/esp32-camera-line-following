# Step 3: capture one snapshot and open it
# Usage: run.bat step3            -> 1280x720
#        run.bat step3 640 480    -> custom size
param(
    [string]$VideoName,
    [int]$W = 1280,
    [int]$H = 720
)

. "$PSScriptRoot\_common.ps1"
Assert-Ffmpeg
$VideoName = Resolve-VideoDevice -Name $VideoName
$outDir = New-OutputDir
$file = Join-Path $outDir ("snap_{0}.jpg" -f (Get-Date -Format "yyyyMMdd_HHmmss"))

Write-Host ("Capturing {0}x{1} from '{2}' ..." -f $W, $H, $VideoName) -ForegroundColor Cyan
# try MJPEG first (standard for this module), fall back to auto codec
& ffmpeg -hide_banner -f dshow -vcodec mjpeg -video_size "$($W)x$($H)" -i "video=$VideoName" -frames:v 1 -y $file 2>&1 | Out-Null
if (-not (Test-Path $file)) {
    Write-Host "  MJPEG capture failed, retrying with auto codec..." -ForegroundColor Yellow
    & ffmpeg -hide_banner -f dshow -video_size "$($W)x$($H)" -i "video=$VideoName" -frames:v 1 -y $file 2>&1 | Out-Null
}
if (Test-Path $file) {
    Write-Host "Saved + opening: $file" -ForegroundColor Green
    Invoke-Item $file
    Write-Host "=== Judgement ===" -ForegroundColor Yellow
    Write-Host "  PASS = image is sharp, no green/garbled blocks, colors look right"
} else {
    Write-Host "[ERROR] capture failed. Check step2 output for supported sizes," -ForegroundColor Red
    Write-Host "  e.g.: run.bat step3 640 480"
}
