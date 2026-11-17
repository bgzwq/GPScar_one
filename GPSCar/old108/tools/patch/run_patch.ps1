# run_patch.ps1  --  ASCII only.
# Applies UTF-8 payload patches onto GB2312/UTF-8 sources without touching the rest.
# Payload format (UTF-8, one .txt may contain several #FILE sections):
#   #FILE <relative path>
#   #ENC  gbk|utf8
#   #OP   after|before|repl|eof  <lineCount>  <ascii find string>
#   <content lines...>
#   #OP   ...
#   #FILE ...
# Rules:
#   - <ascii find string> must occur on exactly ONE line of the target, else the file is skipped.
#   - content indentation: every leading group of 4 spaces becomes one TAB.
#   - ops of one file are applied bottom-up, so earlier line numbers stay valid.
param(
    [string]$PayloadDir = 'D:\table2\GPSCar_PRO\GPSCar\old108\tools\patch\payload',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$root = 'D:\table2\GPSCar_PRO\GPSCar\old108'
$gb   = [Text.Encoding]::GetEncoding(936)
$utf8 = New-Object System.Text.UTF8Encoding($false)
$bom  = [char]0xFEFF
$TAB  = [string][char]9

function Convert-Indent([string]$line, [string]$ind) {
    if ($ind -eq 'space') { return $line }
    if ($line -match '^( +)(.*)$') {
        $n = [int][Math]::Floor($Matches[1].Length / 4)
        $rest = $Matches[1].Substring($n * 4) + $Matches[2]
        return (($TAB * $n) + $rest)
    }
    return $line
}

function Invoke-FilePatch([string]$rel, [string]$enc, [string]$ind, $ops) {
    $path = Join-Path $root $rel
    if (-not (Test-Path $path)) { return "MISSING  $rel" }
    $encObj = if ($enc -eq 'utf8') { $utf8 } else { $gb }
    $raw = [IO.File]::ReadAllText($path, $encObj)
    $nl  = if ($raw.Contains("`r`n")) { "`r`n" } else { "`n" }
    $arr = [System.Collections.ArrayList]@($raw -split [regex]::Escape($nl))
    $origCount = $arr.Count

    $resolved = @()
    foreach ($op in $ops) {
        if ($op.action -eq 'eof') { $op['line'] = $arr.Count; $resolved += $op; continue }
        if ($op.ContainsKey('fixedLine')) {
            $fl = $op.fixedLine
            if ($fl -lt 0 -or $fl -ge $arr.Count -or $arr[$fl] -notmatch [regex]::Escape($op.find)) {
                return ("SKIP     {0}  :: fixed line {1} does not contain '{2}'" -f $rel, ($fl + 1), $op.find)
            }
            $op['line'] = $fl
            $resolved += $op
            continue
        }
        if ($op.action -eq 'replall') {
            $hs = @()
            for ($i = 0; $i -lt $arr.Count; $i++) {
                if ($null -ne $arr[$i] -and $arr[$i].Contains($op.find)) { $hs += $i }
            }
            if ($hs.Count -eq 0) { return ("SKIP     {0}  :: replall not found : '{1}'" -f $rel, $op.find) }
            foreach ($h in $hs) {
                $resolved += @{ action = 'repl'; count = $op.count; find = $op.find; content = $op.content; line = $h }
            }
            continue
        }
        $hit = @()
        for ($i = 0; $i -lt $arr.Count; $i++) {
            if ($null -ne $arr[$i] -and $arr[$i].Contains($op.find)) { $hit += $i }
        }
        if ($hit.Count -ne 1) {
            return ("SKIP     {0}  :: anchor x{1} : '{2}'" -f $rel, $hit.Count, $op.find)
        }
        $op['line'] = $hit[0]
        $resolved += $op
    }

    $resolved = @($resolved | Sort-Object -Property @{ Expression = { $_.line } } -Descending)
    $changes = 0
    foreach ($op in $resolved) {
        $c = @($op.content)
        switch ($op.action) {
            'after'  { $arr.InsertRange($op.line + 1, $c); $changes++ }
            'before' { $arr.InsertRange($op.line, $c);     $changes++ }
            'repl'   {
                $n = [Math]::Max(1, $op.count)
                if (($op.line + $n) -gt $arr.Count) { return ("SKIP     {0} :: repl out of range" -f $rel) }
                $arr.RemoveRange($op.line, $n)
                $arr.InsertRange($op.line, $c)
                $changes++
            }
            'eof'    {
                while ($arr.Count -gt 0 -and ($arr[$arr.Count - 1] -eq '')) { $arr.RemoveAt($arr.Count - 1) }
                $arr.AddRange($c)
                $changes++
            }
            default  { return ("SKIP     {0} :: bad action {1}" -f $rel, $op.action) }
        }
    }

    $outText = ($arr -join $nl)
    if (-not $outText.EndsWith($nl)) { $outText += $nl }

    $bkDir = Join-Path $root 'tools\patch\backup'
    New-Item -ItemType Directory -Path $bkDir -Force | Out-Null
    $bk = Join-Path $bkDir (($rel -replace '[\\/]', '__'))
    [IO.File]::WriteAllText($bk, $raw, $utf8)

    if (-not $DryRun) { [IO.File]::WriteAllText($path, $outText, $encObj) }
    return ("OK       {0}  ops={1} lines {2}->{3} [{4}]" -f $rel, $changes, $origCount, $arr.Count, $enc)
}

# ---------------- 1. parse payloads ----------------
$all = @()
$payloadFiles = @(Get-ChildItem -Path $PayloadDir -Filter *.txt | Sort-Object Name)
if ($payloadFiles.Count -eq 0) { Write-Host "no payload files in $PayloadDir" -ForegroundColor Red; exit 1 }

foreach ($pf in $payloadFiles) {
    $text = [IO.File]::ReadAllText($pf.FullName, $utf8).TrimStart($bom)
    $lines = $text -split "\r?\n"
    $curFile = $null; $curEnc = 'gbk'; $curInd = 'tab'; $ops = @(); $curOp = $null; $content = @()
    foreach ($ln in $lines) {
        if ($ln -match '^#FILE\s+(.+?)\s*$') {
            if ($null -ne $curOp) { $curOp['content'] = $content; $ops += $curOp; $curOp = $null }
            if ($null -ne $curFile) { $all += ,@{ rel = $curFile; enc = $curEnc; ind = $curInd; ops = $ops } }
            $curFile = $Matches[1]; $curEnc = 'gbk'; $curInd = 'tab'; $ops = @(); $content = @()
        }
        elseif ($ln -match '^#IND\s+(.+?)\s*$') { $curInd = $Matches[1].ToLower() }
        elseif ($ln -match '^#ENC\s+(.+?)\s*$') { $curEnc = $Matches[1].ToLower() }
        elseif ($ln -match '^#OP@(\d+)\s+(\w+)\s+(\d+)\s+(.*)$') {
            if ($null -ne $curOp) { $curOp['content'] = $content; $ops += $curOp }
            $curOp = @{ action = $Matches[2]; count = [int]$Matches[3]; find = $Matches[4]; fixedLine = ([int]$Matches[1] - 1); content = @() }
            $content = @()
        }
        elseif ($ln -match '^#OP\s+(\w+)\s+(\d+)\s+(.*)$') {
            if ($null -ne $curOp) { $curOp['content'] = $content; $ops += $curOp }
            $curOp = @{ action = $Matches[1]; count = [int]$Matches[2]; find = $Matches[3]; content = @() }
            $content = @()
        }
        elseif ($null -ne $curOp) { $content += (Convert-Indent $ln $curInd) }
    }
    if ($null -ne $curOp) { $curOp['content'] = $content; $ops += $curOp }
    if ($null -ne $curFile) { $all += ,@{ rel = $curFile; enc = $curEnc; ind = $curInd; ops = $ops } }
}

# ---------------- 2. apply ----------------
$failed = 0
foreach ($item in $all) {
    $r = Invoke-FilePatch $item.rel $item.enc $item.ind $item.ops
    if ($r -like 'OK*') { Write-Host $r -ForegroundColor Green }
    else { Write-Host $r -ForegroundColor Red; $failed++ }
}
Write-Host ("== patch done, files={0} problems={1}" -f $all.Count, $failed)
if ($failed -gt 0) { exit 1 }
exit 0
