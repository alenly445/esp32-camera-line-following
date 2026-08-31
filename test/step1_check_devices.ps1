# Step 1: enumerate camera / microphone / speaker devices (Windows built-in, no ffmpeg needed)
$ErrorActionPreference = "SilentlyContinue"

Write-Host "=== Video devices (Camera / Image class) ===" -ForegroundColor Cyan
$cams = Get-PnpDevice -Class Camera, Image | Where-Object { $_.Status -eq "OK" }
if ($cams) {
    $cams | Format-Table Status, Class, FriendlyName -AutoSize | Out-String -Width 120 | Write-Host
} else {
    Write-Host "  (none found - USB camera not plugged in or not recognized)" -ForegroundColor Red
}

Write-Host "=== Audio endpoints (mic / speaker) ===" -ForegroundColor Cyan
$ep = Get-CimInstance Win32_PnPEntity | Where-Object { $_.PNPClass -eq "AudioEndpoint" }
if ($ep) {
    $ep | Format-Table Name, Status -AutoSize | Out-String -Width 120 | Write-Host
} else {
    Write-Host "  (none found)" -ForegroundColor Red
}

Write-Host "=== Sound hardware (backend devices) ===" -ForegroundColor Cyan
$snd = Get-CimInstance Win32_SoundDevice
if ($snd) {
    $snd | Format-Table Name, Status -AutoSize | Out-String -Width 120 | Write-Host
}

Write-Host "=== Judgement ===" -ForegroundColor Yellow
Write-Host "  - A camera entry that is NOT 'Integrated Camera' -> USB camera enumerated OK"
Write-Host "  - Mic + speaker entries with 'USB Audio' in name  -> 3-in-1 audio path OK"
Write-Host "  - Only Realtek / Integrated entries               -> camera NOT detected:"
Write-Host "    re-plug USB, check all 4 wires (5V/GND/D+/D-), try another USB port"
