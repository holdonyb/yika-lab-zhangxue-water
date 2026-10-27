"""Contact sheets from the new source frames; verification summaries stay scoped."""
from pathlib import Path
import argparse,json,hashlib
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'visuals';OUT=HERE/'rotation_repair_20261027'
def read(p):return json.loads(p.read_text(encoding='utf8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mark-viewed',action='store_true');args=parser.parse_args()
    all_rows=[];font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17)
    for branch in ['base','planing']:
        folder=HERE/('planing/clips' if branch=='planing' else 'clips')
        for job in read(OUT/f'{branch}_jobs.json'):
            stem=folder/job['id'];receipt=stem.with_suffix('.json');d=read(receipt)
            if d['render_receipt'].get('paddle_geometry_sha256')!=sha(HERE/'paddle_geometry.py'):raise RuntimeError('Unrepaired source: '+job['id'])
            all_rows.append((branch,job,receipt,d))
    contacts=[]
    for page,start in enumerate(range(0,len(all_rows),6),1):
        part=all_rows[start:start+6];sheet=Image.new('RGB',(1440,len(part)*294),'#091c2b');draw=ImageDraw.Draw(sheet)
        for row,(_,job,receipt,d) in enumerate(part):
            draw.text((10,row*294+3),job['id'],font=font,fill='#ffd07a')
            frames=Path(d['frames_dir'])
            for col,index in enumerate([1,72,144]):
                with Image.open(frames/f'frame_{index:04d}.png') as image:sheet.paste(image.convert('RGB').resize((480,270),Image.Resampling.LANCZOS),(col*480,row*294+24))
        path=OUT/f'source_review_{page:02d}.jpg';sheet.save(path,quality=94);contacts.append(str(path.relative_to(ROOT)).replace('\\','/'))
    if args.mark_viewed:
        # This flag is used only after the owner has actually opened the sheets.
        for branch,job,receipt,d in all_rows:
            d['visual_review']={'reviewer':'owner model actual view_image inspection','representative_frames_viewed':[1,72,144],
                'scope':'Representative new source frames, plus directed actual-mesh probes and separate continuous final-encode review. No continuous human watch.','video_sha256':d['video_sha256']}
            receipt.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
    still_rows=[]
    for branch,manifest,folder in [('base','still_jobs.json','stills'),('planing','planing_still_jobs.json','planing/stills')]:
        for job in read(HERE/manifest):
            image=HERE/folder/(job['id']+'.png');receipt=image.with_suffix('.json');d=read(receipt)
            image_pass=d['image_decode_pass'] and d['image_sha256']==sha(image)
            if branch=='planing':image_pass=image_pass and read(image.with_suffix('.planing_check.json'))['pass']
            still_rows.append((branch,job,image,receipt,d,bool(image_pass)))
    picture_rows=[(j['id'],image) for _,j,image,_,_,_ in still_rows]
    picture_rows += [(name,HERE/(name+'.png')) for name in ['candidates/motorcycle_cutout','candidates/car_cutout','film_stills/bike_buoyant']]
    still_sheet=Image.new('RGB',(1152,((len(picture_rows)+2)//3)*240),'#091c2b');draw=ImageDraw.Draw(still_sheet)
    small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',12)
    for i,(name,image) in enumerate(picture_rows):
        x=(i%3)*384;y=(i//3)*240
        with Image.open(image) as im:
            rgba=im.convert('RGBA');rgba.thumbnail((384,216),Image.Resampling.LANCZOS);still_sheet.paste(rgba,(x+(384-rgba.width)//2,y+24+(216-rgba.height)//2),rgba)
        draw.text((x+5,y+4),name,font=small,fill='#ffd07a')
    still_sheet.save(OUT/'still_review.jpg',quality=94)
    contacts.append(str((OUT/'still_review.jpg').relative_to(ROOT)).replace('\\','/'))
    if args.mark_viewed:
        for _,_,_,receipt,d,_ in still_rows:
            d['representative_frame_viewed']=True
            d['visual_review']={'reviewer':'owner model actual contact-sheet inspection','scope':'New direction-repaired still geometry','image_sha256':d['image_sha256']}
            receipt.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
    for branch in ['base','planing']:
        rows=[row for row in all_rows if row[0]==branch];checks=[];pos=[];angles=[]
        for _,job,receipt,d in rows:
            c=read(receipt.with_suffix('.planing_check.json' if branch=='planing' else '.pose_check.json'))
            checks.append({'id':job['id'],'pass':c['pass'] if branch=='planing' else c['pose_transfer_pass'] and c['wave_parameter_identity']})
            pos.append(c['max_position_error_m']);angles.append(c['max_orientation_error_rad'])
        target=HERE/('planing/delivery_validation.json' if branch=='planing' else 'delivery_validation.json')
        own_stills=[{'id':j['id'],'pass':passed,'sha256':d['image_sha256']} for b,j,_,_,d,passed in still_rows if b==branch]
        result={'pass':all(c['pass'] for c in checks) and all(c['pass'] for c in own_stills),'scope':'Current film-used direction-repaired source clips and declared stills; old unused sources remain historical.',
            'usable_videos':len(rows),'total_video_duration_s':sum(j['duration'] for _,j,_,_ in rows),'total_video_frames':sum(round(j['duration']*24) for _,j,_,_ in rows),
            'stills':len(own_stills),'still_checks':own_stills,
            'max_position_error_m':max(pos),'max_orientation_error_rad':max(angles),'checks':checks,
            'paddle_geometry_sha256':sha(HERE/'paddle_geometry.py'),'source_contact_sheets':contacts,
            'actual_review':'representative_frames_viewed' if args.mark_viewed else 'pending_actual_view','continuous_human_watch':False}
        target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    (OUT/'source_review.json').write_text(json.dumps({'contacts':contacts,'source_count':len(all_rows),'actual_review':'viewed' if args.mark_viewed else 'pending'},ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'source_count':len(all_rows),'contacts':contacts,'marked_viewed':args.mark_viewed},ensure_ascii=False))

if __name__=='__main__':main()
