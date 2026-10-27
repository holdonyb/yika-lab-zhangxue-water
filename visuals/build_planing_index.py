import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
entries=[]
for manifest,kind,folder,ext in [('planing_jobs.json','video','clips','mp4'),('planing_still_jobs.json','still','stills','png'),('planing_diagnostic_videos.json','video','clips','mp4'),('planing_diagnostic_stills.json','still','stills','png')]:
    for job in json.loads((HERE/manifest).read_text(encoding='utf8')):
        base=HERE/'planing'/folder/job['id'];record=base.with_suffix('.json');audit=base.with_suffix('.planing_check.json')
        d=json.loads(record.read_text(encoding='utf8')) if record.exists() else {}
        c=json.loads(audit.read_text(encoding='utf8')) if audit.exists() else {}
        review=d.get('visual_review',{});review=review if isinstance(review,dict) else {}
        entries.append({'kind':kind,'use_in_film':True,**job,'file':str(base.with_suffix('.'+ext)),'available':bool(d),
                        'receipt':str(record),'check':str(audit),'geometry_pose_water_pass':c.get('pass',False),
                        'full_decode_pass':d.get('full_decode_pass') if kind=='video' else d.get('image_decode_pass'),
                        'anchors_file':str(base.with_suffix('.anchors.json')),
                        'representative_frames_viewed':review.get('representative_frames_viewed',[]) if kind=='video' else d.get('representative_frame_viewed',False)})
(HERE/'planing/delivery_index.json').write_text(json.dumps(entries,ensure_ascii=False,indent=2),encoding='utf8')
lines=['# 滑行测试新增素材','','| 素材 | 状态 | 求解时间 |','|---|---|---|']
for e in entries:
    ready=e['available'] and e['geometry_pose_water_pass'] and e['full_decode_pass'] and bool(e['representative_frames_viewed'])
    span=f"t={e['start']}，{e['scale']}×，输出 {e['duration']} s" if e['kind']=='video' else f"t={e['time']} 静帧"
    state='诊断保留，不入片' if not e['use_in_film'] else ('已验证' if ready else ('待检查' if e['available'] else '待输出'))
    lines.append(f"| [{e['id']}]({Path(e['file']).as_posix()}) | {state} | {span} |")
(HERE/'planing/DELIVERY.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print(json.dumps({'videos_ready':sum(e['available'] for e in entries if e['kind']=='video' and e['use_in_film']),'videos_requested':sum(e['kind']=='video' and e['use_in_film'] for e in entries),'stills_ready':sum(e['available'] for e in entries if e['kind']=='still' and e['use_in_film'])}))
