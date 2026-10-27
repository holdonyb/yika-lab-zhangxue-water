"""Refresh the local handoff index from completed render receipts."""
import json
import hashlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
jobs = sum([json.loads((HERE/f).read_text(encoding='utf8')) for f in ['formal_jobs.json','extra_jobs.json','repair_jobs.json','narrative_jobs.json']],[])
still_jobs = json.loads((HERE/'still_jobs.json').read_text(encoding='utf8'))
excluded = {'bike_flat_final_paddle_t2_s05': '24 fps sampling aliases the repeating paddles; use bike_flat_final_paddle_t2_s01.'}
excluded.update({f'car_{variant}_final_cutaway_t0_s05': f'Water section lower boundary enters the last frames; use car_{variant}_final_cutaway_t0_s05_deepwater.' for variant in ['no_mod','paddles']})
entries = []
for j in jobs:
    base = HERE/'clips'/j['id']
    receipt = base.with_suffix('.json')
    check = base.with_suffix('.pose_check.json')
    record = json.loads(receipt.read_text(encoding='utf8')) if receipt.exists() else {}
    rotation_manifest=HERE/'rotation_repair_20261027/base_jobs.json'
    if rotation_manifest.exists() and j['id'] not in excluded:
        helper_hash=hashlib.sha256((HERE/'paddle_geometry.py').read_bytes()).hexdigest()
        if record.get('render_receipt',{}).get('paddle_geometry_sha256')!=helper_hash:
            excluded[j['id']]='Pre-direction-repair source retained for history; not used in the corrected film.'
    pose = json.loads(check.read_text(encoding='utf8')) if check.exists() else {}
    review = record.get('visual_review', {})
    reviewed = isinstance(review, dict) and bool(review.get('representative_frames_viewed',review.get('viewed_frames')))
    entries.append({'kind': 'video', **j, 'file': str(base.with_suffix('.mp4')), 'available': bool(record),
                    'use_in_film': j['id'] not in excluded, 'exclusion_reason': excluded.get(j['id']),
                    'full_decode_pass': record.get('full_decode_pass', False),
                    'pose_transfer_pass': pose.get('pose_transfer_pass', False),
                    'wave_parameter_identity': pose.get('wave_parameter_identity', False),
                    'representative_frames_viewed': reviewed,
                    'trajectory_sha256': record.get('inputs_sha256', {}).get('trajectory'),
                    'anchors_file': record.get('anchors_file'), 'receipt': str(receipt),
                    'max_position_error_m': pose.get('max_position_error_m'),
                    'max_orientation_error_rad': pose.get('max_orientation_error_rad')})
for j in still_jobs:
    base = HERE/'stills'/j['id']; receipt = base.with_suffix('.json')
    record = json.loads(receipt.read_text(encoding='utf8')) if receipt.exists() else {}
    entries.append({'kind': 'still', **j, 'file': str(base.with_suffix('.png')), 'available': bool(record),
                    'use_in_film': True, 'image_decode_pass': record.get('image_decode_pass', False),
                    'trajectory_sha256': record.get('trajectory_sha256'), 'receipt': str(receipt),
                    'representative_frame_viewed': record.get('representative_frame_viewed', False)})
(HERE/'delivery_index.json').write_text(json.dumps(entries, ensure_ascii=False, indent=2), encoding='utf8')
lines = ['# Blender 素材交付索引', '', '正式 MP4 均为 24 fps、6 秒、无字幕。每段 `.json`、`.anchors.json`、`.frames.json` 与视频同名；`.pose_check.json` 为传递核对结果。', '', '| 素材 | 状态 | 求解时间 | 分辨率 |', '|---|---|---|---|']
for e in entries:
    state = '已输出' if e['available'] else '待输出'
    if not e['use_in_film']: state = '已替换，不入片'
    when = f"t={e['start']}，{e['scale']}×" if e['kind']=='video' else f"t={e['time']} 静帧"
    width = e.get('width',1280)
    lines.append(f"| [{e['id']}]({Path(e['file']).as_posix()}) | {state} | {when} | {width}×{round(width*9/16)} |")
(HERE/'DELIVERY.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print(json.dumps({'videos_ready':sum(e['available'] for e in entries if e['kind']=='video'),
                  'usable_video_target':sum(e['use_in_film'] for e in entries if e['kind']=='video'),
                  'stills_ready':sum(e['available'] for e in entries if e['kind']=='still')},ensure_ascii=False))
