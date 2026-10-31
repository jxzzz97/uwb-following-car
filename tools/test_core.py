"""Compile and execute the actual platform-independent C/C++ control modules."""
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'firmware/esp32/lib/follow_core'
BUILD = ROOT / 'build/core-tests'
BUILD.mkdir(parents=True, exist_ok=True)
sources = [str(p) for p in sorted(CORE.glob('*.cpp'))] + [str(CORE/'ekf_filter.c'),str(ROOT/'tests/core_tests.cpp')]
env = os.environ.copy()
compiler = shutil.which('g++') or shutil.which('clang++')
if compiler:
    executable = BUILD / ('core_tests.exe' if os.name == 'nt' else 'core_tests')
    command = [compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(CORE),*sources,'-o',str(executable)]
else:
    vswhere = Path(os.environ.get('ProgramFiles(x86)',r'C:\Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    if not vswhere.exists():
        raise SystemExit('Install a C++ compiler (GCC, Clang, or Visual Studio C++ Build Tools).')
    install = subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
    vcvars = Path(install)/'VC/Auxiliary/Build/vcvars64.bat'
    # cmd.exe has its own quoting rules; subprocess's argv quoting is for CRT apps.
    command_text = f'cmd.exe /d /s /c ""{vcvars}" >nul && set"'
    output = subprocess.check_output(command_text,text=True,errors='replace')
    for line in output.splitlines():
        if '=' in line and not line.startswith('='):
            key,value=line.split('=',1)
            for old in list(env):
                if old.lower()==key.lower(): del env[old]
            env[key]=value
    compiler = shutil.which('cl.exe',path=env.get('Path',env.get('PATH','')))
    if not compiler: raise SystemExit('Visual Studio C++ compiler was not found.')
    executable=BUILD/'core_tests.exe'
    command=[compiler,'/nologo','/utf-8','/std:c++17','/EHsc','/W4','/WX','/I'+str(CORE),*sources,'/Fe:'+str(executable)]
subprocess.run(command,cwd=BUILD,env=env,check=True)
subprocess.run([str(executable)],cwd=BUILD,env=env,check=True)
