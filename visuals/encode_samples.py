"""Encode and validate the two requested short trajectory previews."""
from pathlib import Path
import json, hashlib, subprocess, argparse
from PIL import Image, ImageDraw

HERE=Path(__file__).resolve().parent
S=HERE/'samples'
p=argparse.ArgumentParser();p.add_argument('--clip',choices=['car','bike','all'],default='all');args=p.parse_args()
def command(a):return subprocess.run(a,check=True,capture_output=True,text=True)
def encode(folder,dest,n):
    frames=list(folder.glob('frame_*.png'))
    if len(frames)!=n:raise RuntimeError(f'{folder}: {len(frames)} frames, expected {n}')
    command(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate','24','-i',str(folder/'frame_%04d.png'),
             '-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)])
def validate(dest,inputs,viewed):
    info=json.loads(command(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(dest)]).stdout)
    command(['ffmpeg','-v','error','-i',str(dest),'-f','null','-'])
    data={'file':str(dest),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'full_decode_pass':True,
          'metadata':info,'input_receipts':[json.loads(x.read_text(encoding='utf8')) for x in inputs],
          'selected_frames_viewed':viewed,'continuous_human_viewing':False}
    dest.with_suffix('.validation.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'file':str(dest),'duration':info['format']['duration'],'full_decode_pass':True}))
def contact(items,dest):
    out=Image.new('RGB',(960,290*len(items)),(10,20,26));d=ImageDraw.Draw(out)
    for row,group in enumerate(items):
        for col,(label,path) in enumerate(group):
            im=Image.open(path).convert('RGB');im.thumbnail((480,270));out.paste(im,(col*480,row*290+20));d.text((col*480+12,row*290+5),label,fill='white')
    out.save(dest,quality=90)
if args.clip in ['car','all']:
    encode(S/'car_front_frames',S/'car_front.mp4',72);encode(S/'car_cutaway_frames',S/'car_cutaway.mp4',72)
    listing=S/'car_concat.txt';listing.write_text("file 'car_front.mp4'\nfile 'car_cutaway.mp4'\n",encoding='utf8')
    dest=S/'car_large_floats_preview.mp4'
    command(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(listing),'-c','copy','-movflags','+faststart',str(dest)])
    validate(dest,[S/'car_front_frames.render.json',S/'car_cutaway_frames.render.json'],['front frame 72','cutaway frame 1','cutaway frame 72'])
    contact([[('front t=0',S/'car_front_frames/frame_0001.png'),('front t=2.958',S/'car_front_frames/frame_0072.png')],
             [('cutaway t=3',S/'car_cutaway_frames/frame_0001.png'),('cutaway t=5.958',S/'car_cutaway_frames/frame_0072.png')]],S/'car_contact.jpg')
if args.clip in ['bike','all']:
    dest=S/'bike_low_torque_preview.mp4';encode(S/'bike_low_torque_fixed_frames',dest,144)
    validate(dest,[S/'bike_low_torque_fixed_frames.render.json'],['frame 1 (contact sheet)','frame 72 (contact sheet)','frame 97','frame 144'])
    contact([[('t=2',S/'bike_low_torque_fixed_frames/frame_0001.png'),('t=4.958',S/'bike_low_torque_fixed_frames/frame_0072.png')],
             [('t=6',S/'bike_low_torque_fixed_frames/frame_0097.png'),('t=7.958',S/'bike_low_torque_fixed_frames/frame_0144.png')]],S/'bike_contact.jpg')
