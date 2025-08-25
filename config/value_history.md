## value_table

| Attrs                  | Try1         | Try6         | Try7         | Try8         | Try9         |
|------------------------|--------------|--------------|--------------|--------------|--------------|
| base_mass              | 1.0          | 1.0          |              |              |              |
| chassis_mass           | 0.3          | 0.3          | 0.5          | 0.5          | 0.3          |
| wheel_mass             | 0.05         | 0.05         | 0.15         | 0.15         | 0.01         |
| caster_mass            | 0.02         | 0.02         | 0.10         | 0.10         | 0.01         |
| wheel_damping          | 0.2          | 0.3          | 1.0          | 1.0          | 1.0          |
| wheel_friction         | 0.04         | 0.04         | 1.0          | 1.0          | 1.0          |
| caster_damping         | 0.0          | 0.0          | 0.0          | 0.0          | 0.0          |
| caster_friction        | 0.1          | 0.1          | 0.1          | 0.1          | 0.1          |
| max_wheel_torque       | 2.0          | 3.0          | 1.0          | 1.0          | 1.0          |
| max_wheel_acceleration | 0.5          | 0.6          | 2.5          | 2.5          | 2.0          |
| max_linear_velocity    | 1.0          | 1.0          | 1.0          | 1.0          | 1.9          |
| max_angular_velocity   | 2.0          | 3.0          | 6.0          | 6.0          | 6.0          |
| drive_mu               | 1.5          | 1.0          | 10.0         | 10.0         | 10.0         |
| caster_mu              | 0.1          | 0.02         | 0.1          | 0.1          | 0.1          |
| drive_slip             | 0.01         | 0.12         | 0.0009       | 0.01         |              |
| caster_slip            | 0.05         | 0.2          | 0.7          | 0.05         |              |
| drive_kp               | 1000.0       | 1000.0       | 1.0e+27      | 1.0e+27      | 1.0e+27      |
| caster_kp              | 1000.0       | 1000.0       | 1.0e+27      | 1.0e+27      | 1.0e+27      |
| drive_kd               | 1.0          | 1.0          | 1.0e+27      | 1.0e+27      | 1.0e+27      |
| caster_kd              | 10.0         | 8.0          | 1.0e+27      | 1.0e+27      | 1.0e+27      |
| base_ixx_factor        | 0.002        | 0.002        |              |              |              |
| base_iyy_factor        | 0.002        | 0.002        |              |              |              |
| base_izz_factor        | 0.003        | 0.003        |              |              |              |
| wheel_ixx_factor       | 0.00001      | 0.00001      | 0.0003451    | 0.0003451    | 0.000003451  |
| wheel_iyy_factor       | 0.00001      | 0.00001      | 0.0003451    | 0.0003451    | 0.000003451  |
| wheel_izz_factor       | 0.00002      | 0.00002      | 0.0006       | 0.0006       | 0.000006     |
| caster_yr_ixx          | 0.0000031184 | 0.0000031184 | 0.00031184   | 0.00031184   | 0.0000031184 |
| caster_yr_iyy          | 0.0000031184 | 0.0000031184 | 0.00031184   | 0.00031184   | 0.0000031184 |
| caster_yr_izz          | 0.000006195  | 0.000006195  | 0.0006195    | 0.0006195    | 0.000006195  |
| caster_p_ixx           | 0.00000826   | 0.00000826   | 0.000826     | 0.000826     | 0.00000826   |
| caster_p_iyy           | 0.00000826   | 0.00000826   | 0.000826     | 0.000826     | 0.00000826   |
| caster_p_izz           | 0.00000826   | 0.00000826   | 0.000826     | 0.000826     | 0.00000826   |
| diff_update_rate       | 60           | 60           | 60           | 60           | 60           |
| jsp_update_rate        | 30           | 40           | 40           | 40           | 40           |


## observation_table
| Attempts | Observation                                                     | Gazebo Crash |
|----------|-----------------------------------------------------------------|--------------|
| Try 1 ✅ | smooth and fast linera movement\. NO angular or rotation        | No           |
| Try 2    | same as try1 - no angular or turns able to be made              | No           |
| Try 3    | No movement - but wheels and caster has no flickering           | No           |
| Try 4    | No movement                                                     | No           |
| Try 5    | try1 + added tf noise + unstable while stationary               | No           |
| Try 6    | try5 + very little tf noise                                     | No           |
| Try 7 IP | copy of 8 {bounded btw try1 and 9}   | No           |
| Try 8 ✅ | has rotation and linear but wierdly a tabular motion instead of straight after turining  | No           |
| Try 9 ✅ |   | No           |
