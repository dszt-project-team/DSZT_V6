# 只读工程自检；不编译、不烧录、不访问硬件、不修改Git。
$ErrorActionPreference = 'Stop'
$fcRoot = [IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$fcMdk = Join-Path $fcRoot 'MDK-ARM'
$fcRootPrefix = $fcRoot + [IO.Path]::DirectorySeparatorChar
[xml]$fcXml = Get-Content -LiteralPath (Join-Path $fcMdk 'DSZT_V6_FC.uvprojx') -Raw
$fcTarget = $fcXml.Project.Targets.Target
if ($fcTarget.TargetName -ne 'DSZT_V6_FC') { throw 'Incorrect target name' }
$fcPaths = @{}
foreach ($fcFile in $fcTarget.Groups.Group.Files.File) {
    if (-not $fcFile.FilePath) { continue }
    $fcResolved = [IO.Path]::GetFullPath((Join-Path $fcMdk $fcFile.FilePath))
    if (-not $fcResolved.StartsWith($fcRootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "External source reference: $fcResolved"
    }
    if (-not (Test-Path -LiteralPath $fcResolved -PathType Leaf)) { throw "Missing source: $fcResolved" }
    if ($fcPaths.ContainsKey($fcResolved)) { throw "Duplicate source: $fcResolved" }
    if ($fcFile.FilePath -match '(?i)sbus|modules[/\\]remote') { throw "Retired RC source: $($fcFile.FilePath)" }
    $fcPaths[$fcResolved] = $true
}
foreach ($fcInclude in $fcTarget.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';')) {
    $fcResolved = [IO.Path]::GetFullPath((Join-Path $fcMdk $fcInclude))
    if (-not $fcResolved.StartsWith($fcRootPrefix, [StringComparison]::OrdinalIgnoreCase) -or
        -not (Test-Path -LiteralPath $fcResolved -PathType Container)) { throw "Invalid include path: $fcInclude" }
}
$fcIoc = Get-Content -LiteralPath (Join-Path $fcRoot 'DSZT_V6_FC.ioc') -Raw
if ($fcIoc -match 'USART1|SBUS|DMA2_Stream2') { throw 'Retired SBUS peripheral remains in IOC' }
if ($fcIoc -notmatch 'NVIC.TimeBaseIP=TIM6' -or $fcIoc -notmatch 'RCC.HSE_VALUE=12000000') {
    throw 'Unexpected HAL timebase or crystal configuration'
}
$fcDocuments = @(
    (Join-Path $fcRoot 'README.md'), (Join-Path $fcRoot 'THIRD_PARTY_NOTICES.md'),
    (Join-Path $fcRoot 'tests/README.md')
) + @(Get-ChildItem -LiteralPath (Join-Path $fcRoot 'Doc') -Filter '*.md' -Recurse | Select-Object -ExpandProperty FullName)
$fcLinks = 0
foreach ($fcDocument in $fcDocuments) {
    $fcText = Get-Content -LiteralPath $fcDocument -Raw
    if (([regex]::Matches($fcText, '(?m)^```').Count % 2) -ne 0) { throw "Unpaired code fence: $fcDocument" }
    foreach ($fcMatch in [regex]::Matches($fcText, '\]\(([^)]+)\)')) {
        $fcLink = $fcMatch.Groups[1].Value.Trim('<', '>')
        if ($fcLink -match '^(https?:|#)') { continue }
        $fcLink = [Uri]::UnescapeDataString(($fcLink -split '#',2)[0])
        if (-not (Test-Path -LiteralPath (Join-Path (Split-Path -Parent $fcDocument) $fcLink))) {
            throw "Broken link in ${fcDocument}: $fcLink"
        }
        $fcLinks++
    }
}
Write-Output "PASS: $($fcPaths.Count) local unique project files, include paths, FC IOC, $fcLinks local document links."
