"""Rebuild evidence tables and figures from completed, immutable trajectories."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
from car_planing_matrix import ROOT,summarize,read
from check_planing_final_results import energy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':11})

def load(name):return np.load(ROOT/'physics/runs'/name/'trajectory.npz')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def compare(a_name,b_name,kind):
    a=load(a_name);b=load(b_name);end=min(a['t'][-1],b['t'][-1]);window=(a['t']>=max(0,end-5))&(a['t']<=end)
    times=a['t'][window]
    metrics={}
    for field,values_a,values_b in [('forward_speed_m_s',a['qvel'][:,0],b['qvel'][:,0]),('z_m',a['qpos'][:,2],b['qpos'][:,2]),('pitch_deg',a['pitch_deg'],b['pitch_deg'])]:
        xa=values_a[window];xb=np.interp(times,b['t'],values_b)
        metrics[field]={'a_mean':float(xa.mean()),'b_mean':float(xb.mean()),'absolute_mean_difference':float(abs(xa.mean()-xb.mean())),'rmse':float(np.sqrt(np.mean((xa-xb)**2)))}
    metrics['support_weight_ratio_difference']=abs(summarize(a_name)['tail_support_weight_ratios']['panel_pressure']-summarize(b_name)['tail_support_weight_ratios']['panel_pressure'])
    speed=metrics['forward_speed_m_s'];relative=speed['absolute_mean_difference']/max(abs(speed['a_mean']),.1)
    acceptable=relative<.05 and metrics['z_m']['absolute_mean_difference']<.03 and metrics['pitch_deg']['absolute_mean_difference']<1. and metrics['support_weight_ratio_difference']<.05
    return {'a':a_name,'b':b_name,'kind':kind,'window_end_s':float(end),'window_duration_s':float(times[-1]-times[0]),'metrics':metrics,'speed_relative_mean_difference':relative,'within_declared_comparison_thresholds':bool(acceptable),'note':'Wet-band comparison is closure sensitivity, not numerical convergence. Mean agreement alone does not validate unmodelled hydrodynamics.'}

def plot_series(names,title,output):
    fig,axes=plt.subplots(2,2,figsize=(12.8,7.2),layout='constrained')
    colors=['#087f9d','#dc8d0d','#b7485a','#4e669f','#669651']
    for (name,label),color in zip(names,colors):
        if not (ROOT/'physics/runs'/name/'metadata.json').exists():continue
        a=load(name);m=read(ROOT/'physics/runs'/name/'metadata.json');w=m['total_mass_kg']*9.81
        axes[0,0].plot(a['t'],a['qvel'][:,0],label=label,color=color,lw=2)
        axes[0,1].plot(a['t'],a['qpos'][:,2],color=color,lw=2)
        axes[1,0].plot(a['t'],a['pitch_deg'],color=color,lw=2)
        axes[1,1].plot(a['t'],a['force'][:,3,2]/w,color=color,lw=1.2,alpha=.85)
    for ax,label in zip(axes.flat,['前进速度 (m/s)','车身参考点高度 (m)','车身纵倾 (°，抬头为正)','板面竖向支撑 / 车重']):
        ax.set_ylabel(label);ax.set_xlabel('自由求解时间 (s)');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=9);fig.suptitle(title,fontsize=15)
    fig.savefig(ROOT/'research'/output,dpi=150);plt.close(fig)

def main():
    names=sorted(p.parent.name for p in (ROOT/'physics/runs').glob('pln_*/metadata.json'))
    rows=[summarize(n) for n in names]
    manifests=[read(p) for p in sorted((ROOT/'research').glob('car_planing_*_manifest.json'))]
    expected=sorted(set(c['name'] for m in manifests for c in m['cases']))
    pending=sorted(set(expected)-set(names))
    comparisons=[]
    base='pln_four_skis_v15_t0_p176'
    for b,kind in [('pln_four_skis_v15_p176_dt05','half timestep'),('pln_four_skis_v15_p176_grid80','double longitudinal grid')]:
        if base in names and b in names:comparisons.append(compare(base,b,kind))
    if all(n in names for n in ['pln_four_skis_v15_p176_grid80','pln_four_skis_v15_p176_grid80_wet6']):
        comparisons.append(compare('pln_four_skis_v15_p176_grid80','pln_four_skis_v15_p176_grid80_wet6','12 to 6 mm wet band'))
    wave_names=['pln_four_skis_v15_p176_head','pln_four_skis_v15_p176_head_dt05']
    wave_check=None
    if all(n in names for n in wave_names):
        wave_check={'comparison':compare(*wave_names,'head wave half timestep'),'energy':[energy(n) for n in wave_names],
            'interpretation':'The same prescribed head wave was repeated at half timestep. Agreement and residual reduction support numerical consistency of this case only; wave model and whole vehicle remain empirically unvalidated.'}
    report={'status':'experiments_pending' if pending else 'registered_experiments_complete','completed_case_count':len(rows),'pending':pending,
            'classification_note':'Sustained-planing label requires >=30s and declared tail tests. Domain screens cover individual surface kinematics, not wakes, wave impacts or empirical validation of the whole car.',
            'cases':rows,'comparisons':comparisons,'head_wave_check':wave_check,
            'source_hashes':{n:sha(ROOT/'physics'/n) for n in ['hydro.py','hydro_planing.py','planing_surface.py']}}
    (ROOT/'research/car_planing_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    plot_series([(f'pln_paddles_v{v}_t3_p44',f'初速 {v} m/s') for v in [5,10,15,20]],'仅加后轮桨叶：增加初速度，能否持续跑住？\n相同结构、正向驱动，后轮合计 44 kW 上限','planing_entry_speeds.png')
    plot_series([(f'pln_four_skis_v15_t0_p{p}',f'后轮合计 {p} kW 上限') for p in [44,88,176]],'四块固定 6° 滑板：相同 15 m/s 入水，改变轮端动力\n功率与扭矩同步按 1 / 2 / 4 倍变化；全部为模型实验','planing_skis_power.png')
    plot_series([(f'pln_four_skis_v{v}_t0_p176',f'初速 {v} m/s') for v in [10,15,20]],'相同滑板与动力：从不同初速进入水面\n10 秒筛选结果；持续性以延长工况为准','planing_skis_speeds.png')
    if 'pln_four_skis_v15_p176_30s' in names:
        plot_series([(n,label) for n,label in [('pln_four_skis_v15_p176_30s','基准 30 秒'),('pln_four_skis_v15_p176_roll1_30s','初始横倾 1°'),('pln_four_skis_v15_p176_off12','第 12 秒断动力')]],'持续滑行、初始扰动和动力中断\n四块前后滑板，全部运动由自由 6DOF 求解','planing_long_and_poweroff.png')
    labels={'submerged':'下潜停止','unstable_attitude':'姿态失稳','transient_or_unconfirmed':'尚未确认为持续','sustained_planing_in_declared_model':'模型内持续动支撑','displacement_or_mixed_floating':'浮航或混合支撑'}
    md=['# 汽车动支撑补充实验','',f'已完成 {len(rows)} 个独立滑行分支工况；待完成 {len(pending)} 个。另有冻结原模型15m/s的7组控制诊断。','',
        '5 m/s 是原先选择的入水工况，不是汽车水上速度上限。新实验分别检查速度、初始姿态、轮端控制、承水面位置/形状、动力与数值分辨率。全部失败与成功保留，不把10秒仍在水面直接算作持续滑行。','',
        '## 方法范围','',
        '本分支对适合的连续平底面使用 Savitsky 1964 动态项及动态压心，另计几何静浮力；它不是完整 Savitsky 船型预测，也未经过实车试验校准。细长浮体保留旧局部压力闭合，因此其运行结果属于模型敏感性。前后滑板尾流互扰、附加质量、喷溅和自由水面反馈未求解；波浪试验仅为给定入射浪下的准稳态探索。','',
        '动力列为两后轮合计上限。44/88/176 kW 分别伴随每轮260/520/1040 Nm；实际功率取决于求解出来的轮速，未强行保持车速。动力倍增是固定整车质量下的独立预算敏感性，不冒充已完成发动机改造。','',
        '几何复核发现最早wide_low/wide_high的浮体内缘与后轮杯桨侧向包络重叠25mm，这些组保留为数值诊断，禁止作为可实施结构或正式动画采用。另建wide_low_clear将浮体中心从y=±1.30移到±1.35m并重算惯量和自由运动。四滑板与杯桨侧向包络保留25mm间隙；这里只是几何间隙检查，不是结构强度验收。','',
        '四滑板每块1.4×0.65×0.04 m，固定6°安装；新增装置按44.1408kg等效计重，整车1468.1408kg。宽薄浮体每只3×0.75×0.10m，两只排水量仍为0.45m³，安装高度另作对照。质量、重心、惯量和相应限制全部记录在各工况params.json。','',
        '## 全部自由运行结果','',
        '| 工况 | 时长 s | 初速 m/s | 动力上限 kW | 末速 m/s | 末段动支撑/重力 | 结果 |','|---|---:|---:|---:|---:|---:|---|']
    for r in rows:md.append(f'| {r["name"]} | {r["duration_s"]:.2f} | {r["initial_speed_m_s"]:g} | {r["total_power_kw"]:g} | {r["end_forward_speed_m_s"]:.2f} | {r["tail_support_weight_ratios"]["panel_pressure"]:.3f} | {labels[r["outcome"]]} |')
    md+=['','末段统计对不足10秒的下潜工况只是失败过程的记录，不是平衡状态。精确窗口、各力分量、域筛选比例、能源账本与轨迹SHA见 car_planing_results.json。','',
        '## 数值与闭合敏感性','',
        '| 对照 | 末段均速差 | 高度均值差 | 纵倾均值差 | 比较门槛 |','|---|---:|---:|---:|---|']
    for c in comparisons:
        met=c['metrics'];md.append(f'| {c["kind"]} | {100*c["speed_relative_mean_difference"]:.2f}% | {1000*met["z_m"]["absolute_mean_difference"]:.2f} mm | {met["pitch_deg"]["absolute_mean_difference"]:.3f}° | {"范围内" if c["within_declared_comparison_thresholds"] else "超出，需保留差异"} |')
    md+=['','门槛在长程结果前登记：均速差<5%、高度均值差<3cm、纵倾均值差<1°、板面支撑占比差<0.05。12→6mm接水带是模型闭合敏感性，不称为网格收敛。','',
        '## 平水、扰动与波浪的区别','',
        '15m/s和10m/s入水的四滑板高动力组都延长到30秒，末段均速约56.9km/h，静浮力约车重的2.9%，板面动支撑约98.5%，其余竖向力补足账本。它们支持的是当前参数模型中的平水对称直行结果。','',
        '初始横倾1°的组虽未翻覆，末5秒车头方向却从约21°继续转到27°，约9°侧滑使全部板面动态载荷超出项目声明的3°来流筛选。没有把这个工况算作已经恢复稳定直行。四种入射浪均完成12秒，但存在外推载荷且没有30秒持续性确认；不将其写成实车高速过浪能力。','',
        'FINAL_PLANING_RESULT_REVIEW.md及planing_final_result_checks.json保留原始能量残差、原净功归一化，以及明确标为48Hz采样估计的正电机功比例。调整归一化不能代替误差核查。','']
    if wave_check:
        c=wave_check['comparison'];a,b=wave_check['energy']
        md+=['### 迎浪半步长复算','',
            f'同一12秒迎浪工况，dt从1ms降到0.5ms；末5秒平均前进速度差为{100*c["speed_relative_mean_difference"]:.4f}%，高度均值差{1000*c["metrics"]["z_m"]["absolute_mean_difference"]:.3f}mm。', '',
            f'原始能量残差从+{a["signed_energy_residual_j"]:.3f}J降至+{b["signed_energy_residual_j"]:.3f}J；原净功归一化从{100*a["original_energy_balance_relative_error"]:.4f}%降至{100*b["original_energy_balance_relative_error"]:.4f}%，两个指标均保留。', '',
            '这补完了该工况的一次步长对照，仍不是完整网格收敛、冲击/尾流验证或真实波浪试验。研究文件保留迎浪计算结果；新增电影口播只采用平水限定结果。','']
    md+=[
        '## 原始证据','',
        '- PLANING_METHOD_REVIEW.md：主论文公式、适用域、完整力矩检查与生产接水带差异。',
        '- planing_surface_benchmark.json：12个独立力/压心台架。',
        '- planing_review_checks.json：6个域筛选探针及生产网格复验。',
        '- car_entry_speed_results.json：冻结原模型的15m/s及轮速控制对照。',
        '- car_planing_*_manifest.json：各阶段在运行前保存的参数。',
        '- physics/runs/pln_*/：真实MuJoCo状态、逐项力/力矩、功率与能量记录。','']
    (ROOT/'research/CAR_PLANING_RESULTS.md').write_text('\n'.join(md),encoding='utf8')
    print(json.dumps({'completed':len(rows),'pending':pending,'comparisons':comparisons},ensure_ascii=False))

if __name__=='__main__':main()
