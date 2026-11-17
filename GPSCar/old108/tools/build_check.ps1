# build_check.ps1 -- 用 ARMCC 5.06 复现 Keil 工程的编译+链接，用于改动后的验证
# 用法:  pwsh -File tools\build_check.ps1
# 说明:  只做验证，不产生固件用于下载；输出 obj 到 build_check\ 目录。
$ErrorActionPreference = 'Stop'

$root   = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$proj   = Join-Path $root 'Project'
$out    = Join-Path $root 'build_check'
$armcc  = 'E:\keil5\MDK\ARM\ARMCC\bin\armcc.exe'
$armasm = 'E:\keil5\MDK\ARM\ARMCC\bin\armasm.exe'
$armlink= 'E:\keil5\MDK\ARM\ARMCC\bin\armlink.exe'

if (-not (Test-Path $armcc)) { Write-Host "armcc not found: $armcc" -ForegroundColor Red; exit 2 }

if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Path $out | Out-Null

$gb = [System.Text.Encoding]::GetEncoding(936)
$xml = [System.IO.File]::ReadAllText((Join-Path $proj 'STM32F407.uvprojx'), $gb)

# ---- 源文件列表（与 uvprojx 完全一致）----
$srcs = [regex]::Matches($xml, '<FilePath>(.*?)</FilePath>') | ForEach-Object { $_.Groups[1].Value }
$srcs = $srcs | Where-Object { $_ -match '\.(c|s)$' }

$incs = ([regex]::Matches($xml, '<IncludePath>(.*?)</IncludePath>') | ForEach-Object { $_.Groups[1].Value }) -join ';'
$incs = $incs -split ';' | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' } | Select-Object -Unique
$defs = ([regex]::Matches($xml, '<Define>(.*?)</Define>') | ForEach-Object { $_.Groups[1].Value }) -join ','
$defs = $defs -split ',' | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' } | Select-Object -Unique

$incArgs = @()
foreach ($i in $incs) {
    $p = $i.Trim()
    if (-not [System.IO.Path]::IsPathRooted($p)) { $p = Join-Path $proj $p }
    $incArgs += '-I'
    $incArgs += ($p -replace '\\', '/')
}
$defArgs = @()
foreach ($d in $defs) { $defArgs += ('-D' + $d.Trim()) }

Write-Host "== sources: $($srcs.Count)  includes: $($incs.Count)  defines: $($defs.Count)"

$objs = @()
$nErr = 0
$nWarn = 0
$sw = [Diagnostics.Stopwatch]::StartNew()

foreach ($s in $srcs) {
    $p = $s.Trim()
    if (-not [System.IO.Path]::IsPathRooted($p)) { $p = Join-Path $proj $p }
    $p = [System.IO.Path]::GetFullPath($p)
    if (-not (Test-Path $p)) { Write-Host "MISSING $p" -ForegroundColor Red; $nErr++; continue }

    $name = [System.IO.Path]::GetFileNameWithoutExtension($p)
    $obj  = Join-Path $out ($name + '.o')
    $objs += $obj

    if ($p -match '\.s$') {
        $args = @('--cpu','Cortex-M4.fp','--pd','SLEEP SETA 1','--pd','__MICROLIB SETA 1','--thumb') + $incArgs + @($p, '-o', $obj)
        $log = Join-Path $out ($name + '.log')
        $txt = & $armasm @($args) 2>&1 | Out-String
        $txt | Set-Content -Encoding utf8 $log
    } else {
        $args = @('--c99','--cpu','Cortex-M4.fp','-O3','-Ospace','--library_type=microlib',
                  '-D__MICROLIB','--apcs=interwork','-c') + $incArgs + $defArgs + @($p, '-o', $obj)
        $log = Join-Path $out ($name + '.log')
        $txt = & $armcc @($args) 2>&1 | Out-String
        $txt | Set-Content -Encoding utf8 $log
    }

    $e = ([regex]::Matches($txt, 'Error:')).Count
    $w = ([regex]::Matches($txt, 'Warning:')).Count
    if ($e -gt 0) {
        $nErr += $e
        Write-Host "ERR  $name" -ForegroundColor Red
        Write-Host $txt
    } elseif ($w -gt 0) {
        $nWarn += $w
        Write-Host "WARN $name" -ForegroundColor Yellow
        Write-Host $txt
    }
}

Write-Host ("== compile done: {0:N1}s  errors={1} warnings={2}" -f $sw.Elapsed.TotalSeconds, $nErr, $nWarn)

if ($nErr -eq 0) {
    # ---- 链接 ----
    $sct = Join-Path $out 'STM32F407.sct'
    @'
LR_IROM1 0x08000000 0x00100000  {
  ER_IROM1 0x08000000 0x00100000  {
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000  {
   .ANY (+RW +ZI)
  }
}
'@ | Set-Content -Encoding ascii $sct

    $axf = Join-Path $out 'STM32F407.axf'
    $maplog = Join-Path $out 'link.log'
    $largs = @('--cpu','Cortex-M4.fp','--library_type=microlib','--strict',
               '--scatter', $sct, '-o', $axf, '--info','sizes','--info','totals','--map',
               '--list', (Join-Path $out 'link.map')) + $objs
    $ltxt = & $armlink @($largs) 2>&1 | Out-String
    $ltxt | Set-Content -Encoding utf8 $maplog
    Write-Host $ltxt
    $le = ([regex]::Matches($ltxt,'Error:')).Count
    $lw = ([regex]::Matches($ltxt,'Warning:')).Count
    Write-Host ("== link: errors={0} warnings={1}" -f $le, $lw) -ForegroundColor Cyan
    if ($le -gt 0) { exit 1 }
}

if ($nErr -gt 0) { exit 1 }
exit 0
