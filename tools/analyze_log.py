"""Generate a standalone SVG/HTML report from the controller's exported CSV."""
import argparse
import csv
import html
import math
from collections import Counter
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=Path)
parser.add_argument('--output',type=Path,default=Path('follow-report.html'))
args=parser.parse_args()
with args.csv.open(encoding='utf-8-sig',newline='') as f:
    reader=csv.DictReader(f)
    required={'tick_ms','raw_distance_cm','filtered_distance_cm','raw_angle_deg','filtered_angle_deg','left_command_rpm','right_command_rpm','state'}
    if not required.issubset(reader.fieldnames or []):raise SystemExit('CSV header does not match the exported controller log.')
    rows=list(reader)
if not rows:raise SystemExit('CSV contains no data rows.')
def number(value):
    try:return float(value)
    except (TypeError,ValueError):return math.nan
ticks=[number(r['tick_ms']) for r in rows]
if not all(math.isfinite(t) for t in ticks):raise SystemExit('Invalid tick_ms values.')
times=[0.0]
for a,b in zip(ticks,ticks[1:]):times.append(times[-1]+((int(b)-int(a))&0xffffffff)/1000)
duration=max(times[-1],.001)
def chart(title,fields,unit):
    data=[[number(r[f]) for r in rows] for f in fields]
    finite=[v for values in data for v in values if math.isfinite(v)]
    low=min(finite or [0]);high=max(finite or [1]);margin=max((high-low)*.12,1)
    low-=margin;high+=margin
    colors=['#1a7a71','#b35d31'];lines=[]
    for values,color in zip(data,colors):
        segment=[]
        for t,v in zip(times,values):
            if not math.isfinite(v):
                if segment:lines.append(f'<polyline points="{" ".join(segment)}" fill="none" stroke="{color}" stroke-width="2"/>');segment=[]
                continue
            segment.append(f'{70+t/duration*900:.2f},{220-(v-low)/(high-low)*175:.2f}')
        if segment:lines.append(f'<polyline points="{" ".join(segment)}" fill="none" stroke="{color}" stroke-width="2"/>')
    return f'''<section><h2>{html.escape(title)}</h2><p>{' / '.join(html.escape(f) for f in fields)} · {unit}</p>
    <svg viewBox="0 0 1000 260" role="img" aria-label="{html.escape(title)}"><path d="M70 35V220H970" fill="none" stroke="#80929f"/>
    <text x="8" y="48">{high:.1f}</text><text x="8" y="220">{low:.1f}</text><text x="70" y="248">0 s</text><text x="900" y="248">{duration:.2f} s</text>{''.join(lines)}</svg></section>'''
counts=Counter(r['state'] for r in rows)
summary=' · '.join(f'{html.escape(k)}: {v}' for k,v in sorted(counts.items()))
report=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>UWB 跟随运行报告</title>
<style>body{{max-width:1080px;margin:40px auto;padding:0 24px;font:15px system-ui;color:#243746;background:#f5f7f8}}section{{background:white;border:1px solid #d7e0e5;border-radius:8px;padding:20px;margin:18px 0}}h1{{font-size:26px}}h2{{font-size:18px}}p{{color:#536674}}svg{{width:100%;font-size:12px;fill:#536674}}</style>
<h1>UWB 跟随运行报告</h1><p>来源：{html.escape(args.csv.name)} · {len(rows)} 帧 · {duration:.2f} 秒</p><p>{summary}</p>
<p>绿色：第一数据序列；棕色：第二数据序列。轮速为下发指令，未知值处断开曲线。</p>
{chart('目标距离',['raw_distance_cm','filtered_distance_cm'],'cm')}
{chart('目标角度',['raw_angle_deg','filtered_angle_deg'],'deg')}
{chart('左右轮指令',['left_command_rpm','right_command_rpm'],'rpm')}</html>'''
args.output.write_text(report,encoding='utf-8')
print(f'Report written: {args.output}')
