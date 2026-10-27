from pathlib import Path
import json,re,hashlib
ROOT=Path(__file__).resolve().parents[1]
original=json.loads((ROOT/'archive_versions/planing_before_audio/script.json').read_text(encoding='utf8'))
draft=(ROOT/'research/PLANING_SCRIPT_DRAFT.md').read_text(encoding='utf8')
replacements={m.group(1):m.group(2).strip() for m in re.finditer(r'^## (?:新增 )?(U\d+[A-Z]?)(?: |：)[^\n]*\n\n(.*?)(?=\n## |\Z)',draft,re.M|re.S)}
expected={'U10','U11','U11A','U11B','U11C','U12','U15','U17'}
assert set(replacements)==expected,set(replacements)
units=[]
for old in original['units']:
    units.append({'id':old['id'],'text':replacements.get(old['id'],old['text'])})
    if old['id']=='U11':units.extend({'id':key,'text':replacements[key]} for key in ['U11A','U11B','U11C'])
text='\n\n'.join(u['text'] for u in units)
assert len(units)==20 and '[待' not in text
data={'title':original['title'],'status':'full_editorial_review_pending','units':units,'narration_text':text,
      'unchanged_unit_ids':[u['id'] for u in units if u['id'] not in expected],
      'source_canonical_sha256':original['narration_sha256'],'draft_sha256':hashlib.sha256(text.encode('utf8')).hexdigest()}
(ROOT/'script_planing_draft.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
(ROOT/'script_planing_draft.txt').write_text(text+'\n',encoding='utf8')
print(json.dumps({'characters':len(text),'units':len(units),'unchanged':data['unchanged_unit_ids']},ensure_ascii=False))
