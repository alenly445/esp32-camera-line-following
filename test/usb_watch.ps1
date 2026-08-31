# Background watcher: log every USB device arrival for 10 minutes
# Usage: powershell -File usb_watch.ps1   (log: test/output/usb_watch.log)
$ErrorActionPreference = "SilentlyContinue"
$log = Join-Path $PSScriptRoot "output\usb_watch.log"
New-Item -ItemType Directory -Force -Path (Join-Path $PSScriptRoot "output") | Out-Null
"watch started $(Get-Date -Format 'HH:mm:ss')" | Out-File $log -Encoding utf8

$before = (Get-PnpDevice -PresentOnly).InstanceId
for ($i = 0; $i -lt 300; $i++) {          # 300 x 2s = 10 min
    Start-Sleep -Seconds 2
    $now = (Get-PnpDevice -PresentOnly).InstanceId
    $new = Compare-Object $before $now | Where-Object SideIndicator -eq '=>' | Select-Object -ExpandProperty InputObject
    foreach ($id in $new) {
        $dev = Get-PnpDevice -InstanceId $id
        $line = "{0}  ARRIVE  status={1}  class={2}  name={3}" -f (Get-Date -Format 'HH:mm:ss'), $dev.Status, $dev.Class, $dev.FriendlyName
        $line | Out-File $log -Append -Encoding utf8
    }
    $before = $now
}
"watch ended $(Get-Date -Format 'HH:mm:ss')" | Out-File $log -Append -Encoding utf8
