param(
    [string]$KeilUv4 = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
New-Item -ItemType Directory -Force -Path (Join-Path $Root "release") | Out-Null

if (-not $KeilUv4) {
    $Candidates = @(
        $env:KEIL_UV4,
        "D:\Keil_v5\UV4\UV4.exe",
        "C:\Keil_v5\UV4\UV4.exe",
        "C:\Program Files (x86)\Keil_v5\UV4\UV4.exe"
    ) | Where-Object { $_ -and (Test-Path -LiteralPath $_) }
    $KeilUv4 = $Candidates | Select-Object -First 1
}

if (-not $KeilUv4 -or -not (Test-Path -LiteralPath $KeilUv4)) {
    throw "Keil UV4.exe not found. Pass -KeilUv4 with the full path."
}

function Invoke-KeilBuild {
    param(
        [string]$Name,
        [string]$ProjectRelativePath,
        [string]$ReleaseFileName
    )

    $Project = Join-Path $Root $ProjectRelativePath
    $KeilDir = Split-Path -Parent $Project
    $ProjectName = Split-Path -Leaf $Project
    $Log = Join-Path $KeilDir "build_latest.log"

    New-Item -ItemType Directory -Force -Path (Join-Path $KeilDir "Objects"), (Join-Path $KeilDir "Listings") | Out-Null

    Write-Host "[$Name] Rebuilding $ProjectName"
    $Arguments = @("-r", ".\$ProjectName", "-j0", "-o", ".\build_latest.log")
    $Process = Start-Process -FilePath $KeilUv4 -ArgumentList $Arguments -WorkingDirectory $KeilDir -WindowStyle Hidden -Wait -PassThru

    if (-not (Test-Path -LiteralPath $Log)) {
        throw "[$Name] Keil did not create build_latest.log"
    }

    $BuildText = Get-Content -LiteralPath $Log -Raw
    # UV4 may return a non-zero process code when a build has warnings, so the
    # authoritative success condition is the error count in its build log.
    if ($BuildText -notmatch "0 Error\(s\)") {
        Get-Content -LiteralPath $Log
        throw "[$Name] Build failed"
    }

    $Hex = Join-Path $KeilDir "Objects\CBU5000V210.hex"
    if (-not (Test-Path -LiteralPath $Hex)) {
        throw "[$Name] Build succeeded but HEX was not generated"
    }

    Copy-Item -LiteralPath $Hex -Destination (Join-Path $Root "release\$ReleaseFileName") -Force
    $Summary = ($BuildText -split "`r?`n" | Where-Object { $_ -match "Error\(s\)|warning and 0 error|Program Size" })
    $Summary | ForEach-Object { Write-Host "[$Name] $_" }
}

Invoke-KeilBuild -Name "Tag" -ProjectRelativePath "tag_firmware\Keil\uwb_rngaoa_initiator.uvprojx" -ReleaseFileName "uwb_tag.hex"
Invoke-KeilBuild -Name "Anchor" -ProjectRelativePath "anchor_firmware\Keil\uwb_rngaoa_responder.uvprojx" -ReleaseFileName "uwb_anchor.hex"

$ReleaseFiles = @(
    (Join-Path $Root "release\uwb_tag.hex")
    (Join-Path $Root "release\uwb_anchor.hex")
)
$HashLines = foreach ($File in $ReleaseFiles) {
    $Hash = Get-FileHash -Algorithm SHA256 -LiteralPath $File
    "$($Hash.Hash)  $([IO.Path]::GetFileName($File))"
}
$ChecksumText = ($HashLines -join "`n") + "`n"
[IO.File]::WriteAllText(
    (Join-Path $Root "release\SHA256SUMS.txt"),
    $ChecksumText,
    [Text.Encoding]::ASCII
)

Write-Host "Build complete. Release files updated in $Root\release"
