# v2 数值结果

全部 25 个最终工况与 7 组时间步／空间离散比较通过已登记判据。

静态大浮筒汽车使用独立延长至 40 s 的轨迹；其他停车工况采用各自停止或平衡时间。表中速度为水平速度，坐标 z 为模型参考点。

| 工况 | 时长 s | 总重 kg | 最大排水/车重 | 末速 km/h | 最大横倾° | 最大俯仰° | 观察结果 |
|---|---:|---:|---:|---:|---:|---:|---|
| bike_cross_final | 12.000 | 242 | 0.8985 | 29.64 | 9.671 | 1.246 | continued_motion |
| bike_cut_power_final | 11.438 | 242 | 0.8985 | 0.64 | 180.000 | 89.898 | submerged |
| bike_flat_final | 12.000 | 242 | 0.8985 | 30.47 | 0.000 | 1.008 | continued_motion |
| bike_head_final | 12.000 | 242 | 0.8985 | 11.63 | 0.000 | 4.731 | continued_motion |
| bike_irregular_final | 12.000 | 242 | 0.8985 | 17.07 | 3.110 | 4.486 | continued_motion |
| bike_light_final | 12.000 | 198 | 1.0982 | 35.56 | 0.000 | 0.852 | continued_motion |
| bike_low_torque_final | 12.000 | 242 | 0.8985 | 0.16 | 0.000 | 74.380 | submerged |
| bike_reserve_buoyancy_final | 12.000 | 252 | 1.8940 | 29.22 | 0.000 | 1.032 | continued_motion |
| bike_roll_recovery_final | 10.000 | 242 | 0.8985 | 30.38 | 3.000 | 0.988 | continued_motion |
| bike_stop_base_final | 6.396 | 242 | 0.8985 | 0.30 | 180.000 | 89.883 | submerged |
| bike_stop_buoyant_final | 15.000 | 252 | 1.8940 | 0.29 | 0.000 | 1.335 | settled_floating |
| bike_swell_final | 12.000 | 242 | 0.8985 | 29.94 | 0.000 | 2.116 | continued_motion |
| bike_wide_final | 12.000 | 248 | 0.9547 | 31.57 | 0.000 | 0.759 | continued_motion |
| car_large_cross_final | 12.000 | 1544 | 1.8693 | 5.97 | 2.950 | 1.458 | continued_motion |
| car_large_floats_final | 12.000 | 1544 | 1.8693 | 6.03 | 0.000 | 1.197 | continued_motion |
| car_large_head_final | 12.000 | 1544 | 1.8693 | 5.61 | 0.000 | 2.675 | continued_motion |
| car_large_irregular_final | 12.000 | 1544 | 1.8693 | 5.61 | 1.684 | 3.419 | continued_motion |
| car_large_swell_final | 12.000 | 1544 | 1.8693 | 5.80 | 0.000 | 2.515 | continued_motion |
| car_no_mod_final | 3.250 | 1400 | 0.1926 | 8.90 | 0.000 | 14.611 | submerged |
| car_paddles_final | 3.125 | 1424 | 0.1893 | 6.61 | 0.000 | 20.578 | submerged |
| car_small_floats_final | 4.500 | 1460 | 0.4929 | 5.22 | 0.000 | 20.856 | submerged |
| car_stop_large_floats_final | 40.000 | 1544 | 1.8693 | 0.04 | 3.000 | 0.903 | settled_floating |
| car_stop_no_mod_final | 2.605 | 1400 | 0.1926 | 1.61 | 3.000 | 0.000 | submerged |
| car_stop_paddles_final | 2.646 | 1424 | 0.1893 | 3.15 | 3.000 | 7.776 | submerged |
| car_stop_small_floats_final | 3.584 | 1460 | 0.4929 | 1.70 | 3.000 | 4.732 | submerged |

## 40 秒静态汽车末段

- 最后 2 s 高度范围：0.00599576 m。
- 最后 2 s 平均竖直速度：0.00261532 m/s。
- 平均浮力/重量：1.00029912。
- 最后 2 秒最大角速度：0.00139480 rad/s。

数值验收和工程闭合模型支持本片所述参数实验；实车结构证据、流体近似、参数假设和适用范围见 ENGINEERING.md。逐项收敛误差及能量账本见 scenario_results.json，完整状态与受力见 physics/runs/*/。
