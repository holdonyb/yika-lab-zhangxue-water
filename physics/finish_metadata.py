from pathlib import Path
p=Path(__file__).with_name('hydro.py')
s=p.read_text(encoding='utf8')
s=s.replace("initial_energy_j=float(.5*e.mass*p['initial_speed']**2+e.mass*G*p['initial_z'])", "initial_energy_j=float(arrays['kinetic_energy_j'][0]+arrays['potential_energy_j'][0])")
s=s.replace("source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())", "source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),clip_source_sha256=hashlib.sha256((ROOT/'clipped_box.py').read_bytes()).hexdigest())")
s=s.replace("[('bike_flat','base','calm')", "[('bike_low_torque','low_torque','calm'),('bike_flat','base','calm')")
s=s.replace("stop='COM below -3.5 m; experiment stopped'", "stop='body reference origin below -3.5 m; experiment stopped'")
p.write_text(s,encoding='utf8')
