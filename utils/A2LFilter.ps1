param(
    [Parameter(Mandatory=$true)]
    [string]$InputFile,
    
    [Parameter(Mandatory=$true)]
    [string]$OutputFile
)

function Read-FileContent {
    param([string]$Path)
    
    return Get-Content -Path $Path -Encoding UTF8
}

function Remove-DynamicLines {
    param([string[]]$Lines)
    
    # 去除尾部空白
    $cleanedLines = $Lines | ForEach-Object { $_.TrimEnd() }
    
    # 去除首行时间戳
    if ($cleanedLines.Count -gt 0 -and $cleanedLines[0] -match '/\*\s*Auto-generated:.*\*/') {
        $cleanedLines = $cleanedLines[1..($cleanedLines.Count - 1)]
    }
    
    # 去除末尾所有空行
    while ($cleanedLines.Count -gt 0 -and $cleanedLines[-1].Trim() -eq "") {
        $cleanedLines = $cleanedLines[0..($cleanedLines.Count - 2)]
    }
    
    return $cleanedLines
}

function Write-FileContent {
    param(
        [string]$Path,
        [string[]]$Lines
    )
    
    $newLinesCleaneded = Remove-DynamicLines -Lines $Lines
    $lastPath = $Path + ".last"
    
    $contentChanged = $true
    if (Test-Path $lastPath) {
        $lastLines = Get-Content -Path $lastPath -Encoding UTF8
        $lastLinesCleaneded = Remove-DynamicLines -Lines $lastLines
        
        # PowerShell 5.1 兼容的比较方式
        $diff = Compare-Object $lastLinesCleaneded $newLinesCleaneded
        if ($diff -eq $null) {
            $contentChanged = $false
        } else {
            $contentChanged = $true
        }
    }
    
    # 不论如何都写入，保持文件带脚本改动
    # PowerShell 5.1 兼容的文件写入方式
    [System.IO.File]::WriteAllLines($Path, $Lines, [System.Text.Encoding]::UTF8)
    
    # 更新.last
    [System.IO.File]::WriteAllLines($lastPath, $Lines, [System.Text.Encoding]::UTF8)
    
    if ($contentChanged) {
        Write-Host "[INFO] Generate: $Path"
    } else {
        Write-Host "Contents already match: $Path"
    }
}

function Parse-Blocks {
    param([string[]]$Lines)
    
    $blocks = @()
    $stack = @()
    
    for ($i = 0; $i -lt $Lines.Count; $i++) {
        $stripped = $Lines[$i].Trim()
        
        if ($stripped.StartsWith("/begin")) {
            $parts = $stripped -split '\s+'
            if ($parts.Count -ge 3) {
                $stackItem = New-Object PSObject -Property @{
                    Keyword = $parts[1]
                    Name = $parts[2]
                    Start = $i
                }
                $stack += $stackItem
            }
        }
        elseif ($stripped.StartsWith("/end") -and $stack.Count -gt 0) {
            $parts = $stripped -split '\s+'
            if ($parts.Count -ge 2) {
                $top = $stack[$stack.Count - 1]
                if ($stack.Count -gt 1) {
                    $stack = $stack[0..($stack.Count - 2)]
                } else {
                    $stack = @()
                }
                
                if ($top.Keyword -eq $parts[1]) {
                    $blockItem = New-Object PSObject -Property @{
                        Keyword = $top.Keyword
                        Name = $top.Name
                        Start = $top.Start
                        End = $i
                    }
                    $blocks += $blockItem
                }
            }
        }
    }
    
    return $blocks
}

function Filter-A2L {
    param(
        [string]$InputPath,
        [string]$OutputPath
    )
    
    $lines = Read-FileContent -Path $InputPath
    $blocks = Parse-Blocks -Lines $lines
    
    $mblocks = $blocks | Where-Object { $_.Keyword -in @("MEASUREMENT", "CHARACTERISTIC") }
    
    # 定义类型到 CompMethod 名称和 printf 格式映射
    $compuMap = @{
        "UBYTE" = @("NO_COMPU_UBYTE", "%3.0")
        "SBYTE" = @("NO_COMPU_SBYTE", "%4.0")
        "UWORD" = @("NO_COMPU_UWORD", "%5.0")
        "SWORD" = @("NO_COMPU_SWORD", "%6.0")
        "ULONG" = @("NO_COMPU_ULONG", "%10.0")
        "SLONG" = @("NO_COMPU_SLONG", "%11.0")
        "FLOAT32_IEEE" = @("NO_COMPU_FLOAT32", "%8.6")
    }
    
    # 跳过模式列表
    $skipPatterns = @(
        '.*\._\d+_\..*',  # 带 indexed 成员的变量
        '.*Table.*',
        '.*Coef.*',
        '.*husart0.*',
        '.*hcan0.*',
        '.*_lut*',
        '.*ccp.*',
        '.*GPIOD_InitStruct.*'
    )
    
    $outLines = @()
    $idx = 0
    
    while ($idx -lt $lines.Count) {
        $line = $lines[$idx]
        $stripped = $line.Trim()
        
        # 注入 CompMethods 定义到 MODULE 开始处，并添加空行
        if ($stripped.StartsWith("/begin MODULE")) {
            $outLines += $line
            $outLines += ""  # MODULE 与首个 COMPU_METHOD 之间空行
            
            foreach ($dtype in $compuMap.Keys) {
                $mname = $compuMap[$dtype][0]
                $fmt = $compuMap[$dtype][1]
                
                $outLines += "    /begin COMPU_METHOD $mname `"Identity $dtype`""
                $outLines += "      RAT_FUNC"
                $outLines += "      `"$fmt`""
                $outLines += "      `"`""
                $outLines += "      COEFFS 0 1 0 0 0 1"
                $outLines += "    /end COMPU_METHOD"
                $outLines += ""  # 每个 COMPU_METHOD 之后空行
            }
            $idx++
            continue
        }
        
        # MEASUREMENT/CHARACTERISTIC 处理
        $block = $mblocks | Where-Object { $_.Start -eq $idx } | Select-Object -First 1
        if ($block) {
            $s = $block.Start
            $e = $block.End
            $name = $block.Name
            
            # 检查是否匹配任何跳过模式
            $shouldSkip = $false
            foreach ($pattern in $skipPatterns) {
                if ($name -match $pattern) {
                    $shouldSkip = $true
                    break
                }
            }
            
            if ($shouldSkip) {
                $idx = $e + 1
                continue
            }
            
            $outLines += ""
            for ($i = $s; $i -le $e; $i++) {
                $currentLine = $lines[$i]
                $matched = $false
                
                foreach ($dtype in $compuMap.Keys) {
                    $pattern = "(\s*)($dtype)\s+NO_COMPU_METHOD(.*)"
                    if ($currentLine -match $pattern) {
                        $indent = $matches[1]
                        $dataType = $matches[2]
                        $rest = $matches[3]
                        $mname = $compuMap[$dtype][0]
                        $outLines += "$indent$dataType $mname$rest"
                        $matched = $true
                        break
                    }
                }
                
                if (-not $matched) {
                    $outLines += $currentLine
                }
            }
            $idx = $e + 1
            continue
        }
        
        # 其它行直接复制
        $outLines += $line
        $idx++
    }
    
    # 插入头时间戳
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $outLines = @("/* Auto-generated: $timestamp */") + $outLines
    
    # 清理多余空行
    $final = @()
    $prevEmpty = $false
    
    foreach ($ln in $outLines) {
        if (-not $ln.Trim()) {
            if (-not $prevEmpty) {
                $final += $ln
            }
            $prevEmpty = $true
        } else {
            $final += $ln
            $prevEmpty = $false
        }
    }
    
    Write-FileContent -Path $OutputPath -Lines $final
}

# 主执行逻辑
if (-not $InputFile -or -not $OutputFile) {
    Write-Host "Usage: A2LFilter.ps1 -InputFile <input.a2l> -OutputFile <output.a2l>"
    Write-Host ""
    Write-Host "Example:"
    Write-Host "  A2LFilter.ps1 -InputFile input.a2l -OutputFile output.a2l"
    exit 1
}

try {
    Filter-A2L -InputPath $InputFile -OutputPath $OutputFile
    Write-Host "A2L filtering completed successfully."
} catch {
    Write-Error "Error processing A2L file: $($_.Exception.Message)"
    exit 1
}
