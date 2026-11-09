# Adds an "Arm Studio" shortcut to the Windows desktop and Start menu.
# The shortcut runs the launcher inside WSL (Ubuntu); windows appear through WSLg.
# Requirements: WSL 2 with Ubuntu, Docker usable from inside WSL, the repo cloned in WSL.
# Run (from the folder of this file): double click install_windows.cmd
param(
    [string]$Distro = "Ubuntu",
    [string]$RepoPath = "~/master-s-thesis"  # path of the repo INSIDE WSL
)
$ErrorActionPreference = "Stop"

$iconDir = Join-Path $env:LOCALAPPDATA "ArmStudio"
New-Item -ItemType Directory -Force -Path $iconDir | Out-Null
$icon = Join-Path $iconDir "arm_studio.ico"
Copy-Item (Join-Path $PSScriptRoot "icons\arm_studio.ico") $icon -Force

$shell = New-Object -ComObject WScript.Shell
$targets = @(
    (Join-Path ([Environment]::GetFolderPath("Desktop")) "Arm Studio.lnk"),
    (Join-Path ([Environment]::GetFolderPath("Programs")) "Arm Studio.lnk")
)
foreach ($path in $targets) {
    $lnk = $shell.CreateShortcut($path)
    $lnk.TargetPath = Join-Path $env:WINDIR "System32\wsl.exe"
    $lnk.Arguments = "-d $Distro -- bash -lc `"$RepoPath/software/launcher/arm_studio.sh`""
    $lnk.IconLocation = $icon
    $lnk.WindowStyle = 7  # minimized: no console window in the way
    $lnk.Description = "Έλεγχος και προσομοίωση του ρομποτικού βραχίονα"
    $lnk.Save()
}
Write-Host "Εγκαταστάθηκε: εικονίδιο «Arm Studio» στην επιφάνεια εργασίας και στο μενού Έναρξη."
