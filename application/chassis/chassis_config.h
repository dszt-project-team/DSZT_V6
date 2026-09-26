/* DSZT_V6 chassis 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef CHASSIS_CONFIG_H
/* 头文件重复包含保护，不是功能开关，请勿修改。 */
#define CHASSIS_CONFIG_H

/* 整车参数确认：当前1允许申请行走，0锁住行走；不绕过遥控、转向、OID健康及回中检查。 */
#define CHASSIS_PARAMETERS_CONFIRMED       1U
/* OID行走输出开关：当前1允许非零目标，0只停车；不关闭心跳/状态维护，也不单独关闭转向。 */
#define CHASSIS_OID_OUTPUT_ENABLE         1U

/* 额外行走限速，单位ERPM：当前0不附加限制；非零时同时钳制手动与飞控上限，只能收紧原上限。 */
#define CHASSIS_OID_COMMISSION_MAX_ERPM     0U
/* OID模式/目标寄存器额外回读：当前1开启，0关闭；只在锁车低速且有总线余量时发起，不影响基础状态查询。 */
#define CHASSIS_OID_READBACK_ENABLE         1U

/* 快速换向反馈等待：当前0关闭，目标直接交给OID内部斜坡；1先提交双零速并等待新鲜低速反馈再放行。 */
#define CHASSIS_OID_REVERSE_GUARD_ENABLE     0U
/* 低速判定窗口，单位ERPM，当前±50；用于可选换向等待和锁车额外回读许可，不产生永久停车锁存。 */
#define CHASSIS_OID_REVERSE_ZERO_ERPM       50L
/* 可选换向等待所需连续双侧新样本组数，当前2组；加大更保守但延迟增加，当前等待开关为0时不参与换向。 */
#define CHASSIS_OID_REVERSE_ZERO_PAIRS       2U
/* 低速样本最大年龄，当前200 ms；用于可选换向等待及锁车回读许可。减小更严格，不是OID基础在线500 ms阈值。 */
#define CHASSIS_OID_REVERSE_STATUS_MAX_AGE_MS 200U
/* 飞控自动控制授权：当前1允许MAIN1/2健康回中后释放，0禁止自动释放；不取消PWM采集或SBUS模式选择。 */
#define CHASSIS_FC_CONTROL_ENABLE         1U

/* 左前轮OID的Modbus地址，当前1；范围1～247，须与驱动器一致且与右侧不同。 */
#define CHASSIS_OID_LEFT_ID                1U
/* 右前轮OID的Modbus地址，当前2；范围1～247，须与驱动器一致且与左侧不同。 */
#define CHASSIS_OID_RIGHT_ID               2U
/* 电机极对数，当前14，供底层机械RPM转ERPM接口使用；当前主链路直接使用ERPM，不在此再次乘极对数。须按电机核对。 */
#define CHASSIS_OID_MOTOR_POLE_PAIRS      14U

/* 左轮目标方向系数，仅取+1或-1；当前+1，方向已由OID适配。改动须核对前进实物方向及换向反馈符号。 */
#define CHASSIS_OID_LEFT_DIRECTION         1
/* 右轮目标方向系数，仅取+1或-1；当前+1，方向已由OID适配，不可因左右对称再盲目取反。 */
#define CHASSIS_OID_RIGHT_DIRECTION        1

/* 前后轴距，单位m，当前0.610即610 mm；参与阿克曼外轮角和前轮路径半径计算，按结构实测修改。 */
#define CHASSIS_WHEELBASE_M                0.610f
/* 前轮距，单位m，当前0.500即500 mm；参与左右转角及速度差比例，按结构实测修改。 */
#define CHASSIS_FRONT_TRACK_M              0.500f
/* 固定后轮距，单位m，当前0.500；仅记录结构尺寸，当前前轴阿克曼算法不读取此值。 */
#define CHASSIS_REAR_TRACK_M               0.500f

/* 零油门满舵内轮目标28°，模型外轮20.320382°、轴角76.125212°/66.915019°；沿用连杆比例，须实测新结构并联查80°/82°保护。 */
#define CHASSIS_MAX_INNER_WHEEL_DEG       28.000f

/* 转向模式：当前0正常双闭环；1仅左开环，2仅右开环。开环仅手动可用，另一轮刹停且禁止OID行走，无闭环角度保护。 */
#define CHASSIS_STEER_CALIBRATION_SIDE       0U

/* 单轮开环编码器健康门：当前0旁路，1要求选中侧健康；不影响正常双闭环强制检查双侧编码器。 */
#define CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH 0U
/* 左侧软件零点确认：当前1有效，0不加载该侧零点并阻止正常闭环；更换编码器/机构后须重新标定。 */
#define CHASSIS_STEER_LEFT_ZERO_CONFIRMED    1U
/* 右侧软件零点确认：当前1有效，0不加载该侧零点并阻止正常闭环；须与右侧真实直行位置一致。 */
#define CHASSIS_STEER_RIGHT_ZERO_CONFIRMED   1U

/* 双侧零点综合门，由左右确认标志逻辑与生成；不要改成固定1来绕过单侧未标定检查。 */
#define CHASSIS_STEER_ZERO_CONFIRMED (CHASSIS_STEER_LEFT_ZERO_CONFIRMED && CHASSIS_STEER_RIGHT_ZERO_CONFIRMED)

/* 转向位置闭环开关：当前1启用，0停止正常转向闭环且禁止行走；单轮开环由独立模式宏控制。 */
#define CHASSIS_STEER_CLOSED_LOOP_ENABLE     1U

/* 连杆角度模型确认：当前1允许转向就绪后申请行走；0阻止行走，但不单独关闭转向位置环。 */
#define CHASSIS_STEER_LINKAGE_CONFIRMED      1U

/* 左编码器直行软件零点，当前3189，12位范围0～4095；改值会改变物理回正位置，不写编码器NVM。 */
#define CHASSIS_STEER_LEFT_ZERO_RAW       3189U
/* 右编码器直行软件零点，当前3073，12位范围0～4095；左右独立标定，不互换，不自动按上电位置取零。 */
#define CHASSIS_STEER_RIGHT_ZERO_RAW      3073U

/* 左编码器相对角极性：当前1取反，0不反；须与电机方向一起确保负反馈，改错可能持续向限位运动。 */
#define CHASSIS_STEER_LEFT_SENSOR_INVERT     1U
/* 右编码器相对角极性：当前1取反，0不反；与右侧正误差电机方向配套核对，不按外观推断。 */
#define CHASSIS_STEER_RIGHT_SENSOR_INVERT    1U
/* 左轴目标减反馈为正时的电机方向状态，当前1；0拉低FR、1释放开漏FR。改变须确保反馈朝目标增加。 */
#define CHASSIS_STEER_LEFT_POSITIVE_REVERSE  1U
/* 右轴正误差对应方向状态，当前1；0拉低FR、1释放开漏FR。与SENSOR_INVERT共同决定负反馈方向。 */
#define CHASSIS_STEER_RIGHT_POSITIVE_REVERSE 1U
/* 单轮开环非零指令的起转占空比，单位‰，当前250；越大点动越强，范围0～CAL_MAX_DUTY。 */
#define CHASSIS_STEER_CAL_MIN_DUTY         250
/* 单轮开环满杆占空比，单位‰，当前450；须≥最小值且≤1000，开环无角度保护，调大须短时点动。 */
#define CHASSIS_STEER_CAL_MAX_DUTY         450
/* 单轮开环换向前停止和切FR后等待，各用当前300 ms；加大更慢，不影响闭环模块的独立50 ms换向等待。 */
#define CHASSIS_STEER_CAL_DIRECTION_MS     300U
/* 转向释放所需连续回中时间，当前200 ms；手动还要求油门中位，时间越大启动等待越长。 */
#define CHASSIS_STEER_RELEASE_CENTER_MS    200U

/* 内轮连杆参考对应的电机输出轴角，当前89.719°；与INNER_REFERENCE_DEG相除得轴角/轮角比例，不是当前满舵目标。 */
#define CHASSIS_STEER_INNER_OUTPUT_DEG      89.719f

/* 内轮连杆映射的参考轮角，当前33°；与89.719°配套，须>0，不能随MAX_INNER_WHEEL_DEG一起缩放。 */
#define CHASSIS_STEER_INNER_REFERENCE_DEG   33.0f
/* 外轮连杆映射的参考轮角，当前25°；与82.325°配套，须>0，不是当前28°内轮所对应的外轮角。 */
#define CHASSIS_STEER_OUTER_REFERENCE_DEG   25.0f
/* 外轮参考轮角25°对应的输出轴角，当前82.325°；修改会改变外轮轴角目标，应按连杆实测标定。 */
#define CHASSIS_STEER_OUTER_OUTPUT_DEG      82.325f

/* 上层输出轴目标保护80°；大于即故障而非钳位。高于最大模型轴角76.125212°约3.87°，不代表机械行程已验证。 */
#define CHASSIS_STEER_TARGET_MAX_DEG        80.0f
/* 输出轴目标最大变化率，当前300°/s，须>0；增大更跟手但更急，独立于OID差速斜坡和位置环PWM变化率。 */
#define CHASSIS_STEER_TARGET_SLEW_DEG_PER_S 300.0f
/* OID差速用归一化转向指令变化率，当前1.6/s，须>0；10 ms最多变化16‰，增大差速跟随更快，不限制公共油门。 */
#define CHASSIS_OID_ACKERMANN_CMD_SLEW_PER_S 1.6f

/* 普通目标的轴角停止误差带，当前3°；绝对误差≤此值停机刹车，减小更精细但可能增加抖动，须小于重启阈值。 */
#define CHASSIS_STEER_POSITION_TOLERANCE_DEG         3.0f
/* 已进入误差带后重新起转的轴角误差阈值，当前5°；大于此值才重启，须>停止误差且<反馈保护，形成防抖迟滞。 */
#define CHASSIS_STEER_POSITION_RESTART_DEG           5.0f
/* 输出轴反馈保护82°，反馈绝对值大于即故障；也是内环目标合法性上限，配合上层80°保护，不能替代机械止挡。 */
#define CHASSIS_STEER_POSITION_MAX_ABS_DEG          82.0f
/* 位置比例增益，单位‰/°，当前22；PWM目标由绝对角误差乘此值再限幅，增大更强但可能过冲，不是PID的积分/微分项。 */
#define CHASSIS_STEER_POSITION_KP_PER_DEG           22.0f
/* 启用专用回中误差带的目标轴角范围，当前±1°；设0禁用专用回中分支，改大将更多近零目标纳入回中策略。 */
#define CHASSIS_STEER_RETURN_CENTER_TARGET_DEG       1.0f
/* 回中分支的停止误差，当前4.5°；须≥普通停止误差，增大可减小中位反复动作但回正精度降低。 */
#define CHASSIS_STEER_RETURN_CENTER_TOLERANCE_DEG    4.5f
/* 回中分支的重启误差，当前7°；须>回中停止误差且<反馈保护，增大使中位保持迟滞更宽。 */
#define CHASSIS_STEER_RETURN_CENTER_RESTART_DEG      7.0f
/* 闭环非零运行的最小PWM目标，当前300‰；用于克服静摩擦，须0～MAX_DUTY，实际输出仍经过变化率限制。 */
#define CHASSIS_STEER_POSITION_MIN_DUTY             300
/* 普通闭环PWM目标上限，当前1000‰；调小降低最大驱动力/响应，须≥MIN_DUTY且≤1000。 */
#define CHASSIS_STEER_POSITION_MAX_DUTY            1000
/* 回中分支PWM目标上限，当前1000‰；范围MIN_DUTY～POSITION_MAX_DUTY，调小可减缓回正。 */
#define CHASSIS_STEER_RETURN_CENTER_MAX_DUTY       1000
/* 位置环每10 ms允许的运行占空比变化量，当前50‰；加大响应更快但更突兀，范围1～1000，保护停止可直接停机。 */
#define CHASSIS_STEER_DUTY_RAMP_STEP                  50

/* 随速度指令收小转角开关：当前1开启，0保持1倍；同时作用于转向及差速模型，不依赖实际速度反馈。 */
#define CHASSIS_ACKERMANN_SPEED_GAIN_ENABLE           1U
/* 满参考速度指令下最小转向增益，当前0.70，范围(0,1]；越小高速转角越小，设1相当于不收角。 */
#define CHASSIS_ACKERMANN_SPEED_GAIN_MIN              0.70f
/* 增益降到最小值所需归一化行走量，当前1000‰，须>0；越小越早收角，输入已计入CH7及速度参考值。 */
#define CHASSIS_ACKERMANN_SPEED_GAIN_FULL_COMMAND  1000.0f
/* 速度指令归一化参考，当前4900 ERPM，须≥1；不是额外速度上限，改小会更早收小转角。 */
#define CHASSIS_ACKERMANN_SPEED_REFERENCE_ERPM      4900U
/* 增益调度前后转向量死区，当前0.02即2%，范围[0,1)；加大减少中位差速/转向动作，不是CH1等效脉宽死区。 */
#define CHASSIS_ACKERMANN_COMMAND_DEADBAND            0.02f

#if CHASSIS_STEER_CALIBRATION_SIDE > 2U
#error "Select at most one V6 front steering calibration side"
#endif
#if CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH > 1U
#error "Steering calibration encoder-health gate must be 0 or 1"
#endif
#if CHASSIS_STEER_LEFT_ZERO_RAW > 4095U || CHASSIS_STEER_RIGHT_ZERO_RAW > 4095U
#error "MT6826S zero must be a 12-bit raw value"
#endif
#if CHASSIS_STEER_LEFT_ZERO_CONFIRMED > 1U || CHASSIS_STEER_RIGHT_ZERO_CONFIRMED > 1U
#error "Individual steering zero confirmation must be 0 or 1"
#endif
#if CHASSIS_STEER_CLOSED_LOOP_ENABLE > 1U || CHASSIS_STEER_LEFT_SENSOR_INVERT > 1U || CHASSIS_STEER_RIGHT_SENSOR_INVERT > 1U || CHASSIS_STEER_LEFT_POSITIVE_REVERSE > 1U || CHASSIS_STEER_RIGHT_POSITIVE_REVERSE > 1U
#error "Steering enable and polarity settings must be 0 or 1"
#endif
#if CHASSIS_OID_LEFT_ID < 1U || CHASSIS_OID_LEFT_ID > 247U || CHASSIS_OID_RIGHT_ID < 1U || CHASSIS_OID_RIGHT_ID > 247U || CHASSIS_OID_LEFT_ID == CHASSIS_OID_RIGHT_ID
#error "OID IDs must be distinct Modbus unicast addresses"
#endif
#if (CHASSIS_OID_LEFT_DIRECTION != 1 && CHASSIS_OID_LEFT_DIRECTION != -1) || (CHASSIS_OID_RIGHT_DIRECTION != 1 && CHASSIS_OID_RIGHT_DIRECTION != -1)
#error "OID direction must be +1 or -1"
#endif
#if CHASSIS_ACKERMANN_SPEED_REFERENCE_ERPM < 1U || CHASSIS_STEER_CAL_MIN_DUTY < 0 || CHASSIS_STEER_CAL_MAX_DUTY > 1000 || CHASSIS_STEER_CAL_MIN_DUTY > CHASSIS_STEER_CAL_MAX_DUTY
#error "Invalid steering or speed scheduling parameters"
#endif

/* 飞控有效脉宽下限，当前800 μs；低于此值判范围异常并撤销自动释放，不映射成满幅目标。 */
#define CHASSIS_FC_PWM_MIN_VALID_US        800U
/* 飞控有效脉宽上限，当前2200 μs；高于此值判范围异常。覆盖现MAIN1/2标定端点，修改须匹配真实PWM。 */
#define CHASSIS_FC_PWM_MAX_VALID_US       2200U
/* 飞控有效脉冲超时，当前30 ms；超时撤销自动释放并重置滤波预热，改大增加容忍也延迟掉线停车。 */
#define CHASSIS_FC_PWM_TIMEOUT_MS            30U
/* 飞控非法脉冲范围故障保留时间，当前5 ms；须>0且<输入超时，避免短暂坏脉冲被立即掩盖。 */
#define CHASSIS_FC_PWM_TRANSIENT_HOLD_MS      5U
/* MAIN1行走均值窗口，当前1等于最新有效脉宽直通，范围1～8；设大于1会重新引入油门延迟。 */
#define CHASSIS_FC_DRIVE_AVERAGE_WINDOW       1U
/* MAIN2转向均值窗口，当前8个样本，范围1～8；独立于MAIN1，增大更平滑也增加转向输入延迟。 */
#define CHASSIS_FC_STEER_AVERAGE_WINDOW       8U
/* 飞控上电/异常恢复在线预热，当前4个连续有效样本，范围1～255，独立于均值窗口；正常在线后不额外延迟每次输入。 */
#define CHASSIS_FC_PWM_VALID_TO_ONLINE        4U

/* MAIN1反向满幅端点，当前1260 μs；低于等于此值钳位−1000‰，须低于中位减死区。 */
#define CHASSIS_FC_DRIVE_REVERSE_FULL_US   1260U
/* MAIN1油门中位，当前1502 μs；按飞控实际中位标定，不与遥控CH3中位混用。 */
#define CHASSIS_FC_DRIVE_NEUTRAL_US        1502U
/* MAIN1正向满幅端点，当前1770 μs；高于等于此值钳位+1000‰，须高于中位加死区。 */
#define CHASSIS_FC_DRIVE_FORWARD_FULL_US   1770U
/* MAIN1中位两侧死区半宽，当前50 μs；加大更易保持零速但小油门行程减少，须留出两端映射区间。 */
#define CHASSIS_FC_DRIVE_DEADBAND_US         50U
/* MAIN2左转满幅端点，当前1000 μs，对应−1000‰；须低于中位减死区。 */
#define CHASSIS_FC_STEER_LEFT_FULL_US      1000U
/* MAIN2转向中位，当前1500 μs；修改只改变飞控转向映射，不改变编码器软件零点。 */
#define CHASSIS_FC_STEER_NEUTRAL_US        1500U
/* MAIN2右转满幅端点，当前2000 μs，对应+1000‰；须高于中位加死区。 */
#define CHASSIS_FC_STEER_RIGHT_FULL_US     2000U
/* MAIN2中位死区半宽，当前15 μs；加大减少中位转向动作，但会损失小角度输入范围。 */
#define CHASSIS_FC_STEER_DEADBAND_US         15U
/* 自动模式两路健康且双轴连续回中释放时间，当前200 ms；故障或退出自动模式后重新计时。 */
#define CHASSIS_FC_RELEASE_CENTER_MS         200U
/* 自动模式基础速度上限，当前4900 ERPM，不使用CH7；仍可被额外限速收紧，不应超过OID已配置能力。 */
#define CHASSIS_FC_SPEED_LIMIT_ERPM         4900U

/* 底盘目标、位置环及上层安全控制更新间隔，当前10 ms；OID收发每轮独立服务。改动需联查PWM斜坡、滤波和任务负载。 */
#define CHASSIS_CONTROL_PERIOD_MS           10U

#if CHASSIS_FC_DRIVE_AVERAGE_WINDOW < 1U || CHASSIS_FC_DRIVE_AVERAGE_WINDOW > 8U || CHASSIS_FC_STEER_AVERAGE_WINDOW < 1U || CHASSIS_FC_STEER_AVERAGE_WINDOW > 8U || CHASSIS_FC_PWM_VALID_TO_ONLINE < 1U || CHASSIS_FC_PWM_VALID_TO_ONLINE > 255U
#error "Invalid independent FC averaging or warmup configuration"
#endif
#if CHASSIS_FC_PWM_MIN_VALID_US >= CHASSIS_FC_PWM_MAX_VALID_US || CHASSIS_FC_PWM_MAX_VALID_US > 65535U || CHASSIS_FC_PWM_TRANSIENT_HOLD_MS < 1U || CHASSIS_FC_PWM_TRANSIENT_HOLD_MS >= CHASSIS_FC_PWM_TIMEOUT_MS
#error "Invalid FC pulse validity or timeout configuration"
#endif
#if CHASSIS_FC_DRIVE_REVERSE_FULL_US < CHASSIS_FC_PWM_MIN_VALID_US || CHASSIS_FC_DRIVE_FORWARD_FULL_US > CHASSIS_FC_PWM_MAX_VALID_US || CHASSIS_FC_DRIVE_REVERSE_FULL_US + CHASSIS_FC_DRIVE_DEADBAND_US >= CHASSIS_FC_DRIVE_NEUTRAL_US || CHASSIS_FC_DRIVE_NEUTRAL_US + CHASSIS_FC_DRIVE_DEADBAND_US >= CHASSIS_FC_DRIVE_FORWARD_FULL_US
#error "Invalid MAIN1 endpoints or deadband"
#endif
#if CHASSIS_FC_STEER_LEFT_FULL_US < CHASSIS_FC_PWM_MIN_VALID_US || CHASSIS_FC_STEER_RIGHT_FULL_US > CHASSIS_FC_PWM_MAX_VALID_US || CHASSIS_FC_STEER_LEFT_FULL_US + CHASSIS_FC_STEER_DEADBAND_US >= CHASSIS_FC_STEER_NEUTRAL_US || CHASSIS_FC_STEER_NEUTRAL_US + CHASSIS_FC_STEER_DEADBAND_US >= CHASSIS_FC_STEER_RIGHT_FULL_US
#error "Invalid MAIN2 endpoints or deadband"
#endif

#endif
