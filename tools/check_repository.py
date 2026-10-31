"""Check source packaging, relative documentation links and Keil inputs."""
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
IGNORED={'.git','.pio','build','__pycache__','Objects','Listings'}
paths=[p for p in ROOT.rglob('*') if p.is_file() and not (set(p.relative_to(ROOT).parts)&IGNORED)]
errors=[]
for p in paths:
    rel=p.relative_to(ROOT).as_posix()
    if p.name in {'local_config.h','local_wifi_config.h'}:
        # A local working file is allowed, but it must never be tracked.
        tracked=subprocess.run(['git','ls-files','--error-unmatch',rel],cwd=ROOT,capture_output=True).returncode==0
        if tracked: errors.append(f'Tracked local configuration: {rel}')
        continue
    if p.suffix.lower() not in {'.md','.py','.h','.c','.cpp','.ini','.yml','.txt'}:continue
    text=p.read_text(encoding='utf-8',errors='replace')
    # Split regex tokens so the scanner does not match its own source text.
    if re.search(r'gh[pousr]_'+'[A-Za-z0-9]{30,}|github_'+'pat_[A-Za-z0-9_]{30,}|-----BEGIN '+'(?:RSA |EC |OPENSSH )?PRIVATE KEY-----',text):
        errors.append(f'Potential credential material: {rel}')
    # Vendor snapshot documentation references its original full SDK layout.
    if p.suffix=='.md' and '/shared_sdk/' not in '/'+rel:
        for link in re.findall(r'\]\(([^)]+)\)',text):
            if '://' in link or link.startswith(('#','mailto:')):continue
            target=link.split('#',1)[0]
            if target and not (p.parent/target).exists():errors.append(f'Broken link in {rel}: {target}')
uwb=ROOT/'firmware/uwb_cbu5000v210'
for project in uwb.rglob('*.uvprojx'):
    for node in ET.parse(project).iter('FilePath'):
        rel=(node.text or '').replace('\\','/')
        if '/Listings/' in '/'+rel or '/Objects/' in '/'+rel:continue
        if not (project.parent/rel).exists():errors.append(f'Missing Keil input: {project.name}: {rel}')
if errors:raise SystemExit('\n'.join(errors))
print(f'PASS repository checks: {len(paths)} source files, documentation links and Keil inputs')
