# Windows (MSIX 版 Python) ではパス長制限を避けるため短いパスにコピーして実行する
$src = $PSScriptRoot
$T = "$env:TEMP\llmnav_t"
if (Test-Path $T) { Remove-Item -Recurse -Force $T -Confirm:$false }
robocopy $src $T /E /NFL /NDL /NJH /NJS | Out-Null
$env:PYTHONIOENCODING = "utf-8"
Push-Location $T
python -c "import sys,traceback; sys.path[:0]=[r'$T', r'$T\test']; import test_core as t; f=0
for n in sorted(n for n in dir(t) if n.startswith('test_')):
    try:
        getattr(t,n)(); print('PASS',n)
    except Exception as e:
        f+=1; print('FAIL',n,repr(e)); traceback.print_exc(limit=3)
print('fails',f); sys.exit(1 if f else 0)"
$code = $LASTEXITCODE
Pop-Location
Remove-Item -Recurse -Force $T -Confirm:$false
exit $code
