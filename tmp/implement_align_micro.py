from pathlib import Path
import shutil, subprocess, sys, time, os
ROOT=Path('E:/Inte_handing/car_code1')
files=['Core/Inc/arm_config.h','Core/Src/arm_work.c','tests/arm_work/harness.c','tests/arm_work/run_tests.py','README_arm.md','README_arm_vision_test.md']
backup=ROOT/'tmp'/('before_align_micro_'+time.strftime('%Y%m%d_%H%M%S'))
for name in files:
    dest=backup/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
def edit(name, changes, encoding='gb18030'):
    p=ROOT/name;raw=p.read_bytes();nl=b'\r\n' if b'\r\n' in raw else b'\n';s=raw.replace(b'\r\n',b'\n').decode(encoding)
    for old,new in changes:
        assert old in s,(name,old[:90]);s=s.replace(old,new,1)
    p.write_bytes(s.encode(encoding).replace(b'\n',nl))
edit('Core/Inc/arm_config.h',[
('#define ARM_TRANSPORT_YAW_CDEG', '''/* ALIGN-only micro corrections; motor centidegrees, NOT mechanism degrees.
 * yaw 1 output degree = 300 motor cdeg (3:1 reduction).
 * reach 4 mm = floor(4 * 36000 / 113.1) = 1273 motor cdeg.
 * Hold Z; endpoints remain above the yaw height floor and in the front
 * 60 mm of reach travel. Larger changes use the complete clearance path.
 */
#define ARM_ALIGN_MICRO_YAW_STEP_CDEG   300L
#define ARM_ALIGN_MICRO_REACH_STEP_CDEG 1273L
#define ARM_ALIGN_MICRO_MIN_Z_CDEG      ARM_YAW_MIN_Z_CDEG
#define ARM_ALIGN_MICRO_MIN_REACH_CDEG  ARM_CLEAR_REACH_CDEG
#define ARM_TRANSPORT_YAW_CDEG'''),
('90000L /* 10 mm down at 40 mm/motor turn */','90000L /* 100 mm down at 40 mm/motor turn */')])
edit('Core/Src/arm_work.c',[
('static uint32_t plan_done_state;','static uint32_t plan_done_state;\nstatic uint8_t plan_micro;\nstatic int32_t micro_start[3];'),
('    plan_count=count; plan_index=moving=0U; plan_done_state=after;','    plan_count=count; plan_index=moving=plan_micro=0U; plan_done_state=after;'),
('/* All high-level yaw moves use this same measured clearance pose. */','/* Full-pose yaw moves use clearance; ALIGN micro moves have a separate gate. */'),
('static uint8_t transport_plan(uint32_t after)', '''static int64_t distance_cdeg(int32_t a,int32_t b)
{
    int64_t d=(int64_t)a-b;return d<0?-d:d;
}
static uint8_t micro_pose_ok(const int32_t *p)
{
    uint8_t i;
    for(i=0U;i<3U;i++)if(!pose_ok(i,p[i]))return 0U;
    return p[ARM_Z]>=ARM_ALIGN_MICRO_MIN_Z_CDEG &&
        p[ARM_REACH]>=ARM_ALIGN_MICRO_MIN_REACH_CDEG;
}
/* Return 1 for a bounded same-height ALIGN, otherwise retain full avoidance. */
static uint8_t align_plan(const int32_t *pose)
{
    uint8_t axes[2],i,n=0U;int32_t current[3],targets[2];
    for(i=0U;i<3U;i++)current[i]=arm_motor_status[i].position_cdeg;
    if(!micro_pose_ok(current) || !micro_pose_ok(pose) ||
       distance_cdeg(pose[ARM_Z],current[ARM_Z])>arm_motor_config[ARM_Z].tolerance_cdeg ||
       distance_cdeg(pose[ARM_YAW],current[ARM_YAW])>ARM_ALIGN_MICRO_YAW_STEP_CDEG ||
       distance_cdeg(pose[ARM_REACH],current[ARM_REACH])>ARM_ALIGN_MICRO_REACH_STEP_CDEG)
        return pose_plan(pose,ARM_WORK_ALIGNED);
    /* Serial yaw then reach; never emit a Z move for visual micro correction. */
    for(i=ARM_YAW;i<=ARM_REACH;i++) {
        if(distance_cdeg(pose[i],current[i])>arm_motor_config[i].tolerance_cdeg) {
            axes[n]=i;targets[n++]=pose[i];
        }
    }
    plan(n,axes,targets,ARM_WORK_ALIGNED);plan_micro=1U;
    for(i=0U;i<3U;i++) {
        micro_start[i]=current[i];arm_work_status.target_cdeg[i]=pose[i];
    }
    arm_work_status.target_cdeg[ARM_Z]=current[ARM_Z];
    return 1U;
}
/* Recheck at execution, including the reach step after yaw has completed.
 * A changed pose fails the command; never silently convert mid-plan to a
 * large avoidance move while the visual client expects a small correction.
 */
static uint8_t micro_clear_now(uint8_t axis,int32_t target)
{
    uint8_t i;int32_t current[3];
    for(i=0U;i<3U;i++)current[i]=arm_motor_status[i].position_cdeg;
    if(!micro_pose_ok(current) ||
       distance_cdeg(current[ARM_Z],micro_start[ARM_Z])>arm_motor_config[ARM_Z].tolerance_cdeg ||
       distance_cdeg(current[ARM_YAW],micro_start[ARM_YAW])>ARM_ALIGN_MICRO_YAW_STEP_CDEG ||
       distance_cdeg(current[ARM_REACH],micro_start[ARM_REACH])>ARM_ALIGN_MICRO_REACH_STEP_CDEG)
        return 0U;
    if(axis==ARM_YAW) return pose_ok(axis,target) &&
        distance_cdeg(target,current[axis])<=ARM_ALIGN_MICRO_YAW_STEP_CDEG;
    if(axis==ARM_REACH) return pose_ok(axis,target) && target>=ARM_ALIGN_MICRO_MIN_REACH_CDEG &&
        distance_cdeg(target,current[axis])<=ARM_ALIGN_MICRO_REACH_STEP_CDEG;
    return 0U;
}
static uint8_t transport_plan(uint32_t after)'''),
('if(!pose_plan(pose,ARM_WORK_ALIGNED))return ROUTE_BAD_VALUE;','if(!align_plan(pose))return ROUTE_BAD_VALUE;'),
('remember(p);arm_work_status.aligned_command_id=0U;state(ARM_WORK_ALIGNING);','''remember(p);arm_work_status.aligned_command_id=0U;state(ARM_WORK_ALIGNING);
        if(!plan_count) {
            arm_work_status.aligned_command_id=command;
            arm_work_status.active_axis=255U;state(ARM_WORK_ALIGNED);
        }'''),
('    if(axis==ARM_YAW && !yaw_clear_now()) {fail(ARM_WORK_POSE,axis);return;}','''    if(plan_micro) {
        if(!micro_clear_now(axis,plan_target[plan_index])) {fail(ARM_WORK_POSE,axis);return;}
    } else if(axis==ARM_YAW && !yaw_clear_now()) {fail(ARM_WORK_POSE,axis);return;}''')])
# Extend actual CAN command observations with per-axis motion counts.
edit('tests/arm_work/harness.c',[
('static uint32_t yaw_clearance_violations;','static uint32_t yaw_clearance_violations;\nstatic uint32_t axis_moves[3];'),
('        move_count++;','        move_count++;axis_moves[i]++;'),
('int run_case(uint32_t test)','''static RouteResult_t micro_command(uint32_t id,int32_t z,int32_t yaw,int32_t reach)
{
#if ARM_VISION_TEST_MODE
    return test_command(1,102,id,2,z,yaw,reach,0);
#else
    return command(id,2,z,yaw,reach,0);
#endif
}
static int micro_case(uint32_t test)
{
    uint32_t before,z_before,reach_before,yaw_before;int32_t z,yaw,reach;
    CHECK(home(),700);
#if ARM_VISION_TEST_MODE
    CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_OK && until(ARM_WORK_WAIT_VISION),701);
#else
    CHECK(station(102),701);
#endif
    if(test==77 || test==78 || test==79) {
        int32_t initial_z=test==77?-95000:-1000;
        int32_t initial_reach=test==78?-25000:-3000;
        CHECK(micro_command(10,initial_z,2000,initial_reach)==ROUTE_OK && until(ARM_WORK_ALIGNED),702);
    }
    z=arm_motor_status[0].position_cdeg;yaw=arm_motor_status[1].position_cdeg;
    reach=arm_motor_status[2].position_cdeg;
    before=move_count;z_before=axis_moves[0];yaw_before=axis_moves[1];reach_before=axis_moves[2];
    if(test==70) { /* Both axes, at the output 1 degree / 4 mm limits. */
        CHECK(micro_command(11,z,yaw+300,reach-1273)==ROUTE_OK && until(ARM_WORK_ALIGNED),710);
        CHECK(move_count==before+2 && axis_moves[0]==z_before,711);
        CHECK(axis_moves[1]==yaw_before+1 && axis_moves[2]==reach_before+1,712);
        CHECK(arm_motor_status[2].position_cdeg>-1400,713);
    } else if(test==71 || test==72) {
        CHECK(micro_command(11,z,yaw+(test==71?200:0),reach-(test==72?600:0))==ROUTE_OK && until(ARM_WORK_ALIGNED),720);
        CHECK(move_count==before+1 && axis_moves[0]==z_before,721);
        CHECK(axis_moves[1]==yaw_before+(test==71) && axis_moves[2]==reach_before+(test==72),722);
    } else if(test==73) {
        CHECK(micro_command(11,z,yaw,reach)==ROUTE_OK && until(ARM_WORK_ALIGNED),730);
        CHECK(move_count==before && arm_work_status.aligned_command_id==11,731);
        CHECK(micro_command(11,z,yaw,reach)==ROUTE_OK && move_count==before,732);
    } else if(test==74 || test==75 || test==76 || test==77 || test==78) {
        CHECK(micro_command(11,z-(test==76?900:0),yaw+(test==74?301:200),reach-(test==75?1274:600))==ROUTE_OK && until(ARM_WORK_ALIGNED),740);
        CHECK(move_count==before+5 && axis_moves[0]==z_before+2,741);
    } else if(test==79) { /* Negative yaw / forward reach correction also direct. */
        CHECK(micro_command(11,z,yaw-200,reach+600)==ROUTE_OK && until(ARM_WORK_ALIGNED),790);
        CHECK(move_count==before+2 && axis_moves[0]==z_before,791);
    } else if(test==80 || test==81) { /* Execution-time validation is independent of full clearance. */
        CHECK(micro_command(11,z,yaw+200,reach-600)==ROUTE_OK,800);
        if(test==80)arm_motor_status[0].position_cdeg=ARM_ALIGN_MICRO_MIN_Z_CDEG-1;
        else arm_motor_status[1].position_cdeg=yaw-500;
        step(1);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_POSE,801);
        CHECK(move_count==before,802);return 0;
    } else if(test==82) {
        CHECK(micro_command(11,z,yaw+200,reach-600)==ROUTE_OK && until(ARM_WORK_ALIGNED),820);
        before=move_count;
#if ARM_VISION_TEST_MODE
        CHECK(test_command(1,102,12,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),821);
#else
        CHECK(command(12,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),821);
#endif
        CHECK(move_count==before+5,822);
        CHECK(distance_cdeg_dummy,823);
    }
    CHECK(arm_work_status.fault==ARM_WORK_NO_FAULT,799);return 0;
}
int run_case(uint32_t test)'''),
('CHECK(distance_cdeg_dummy,823);','CHECK(arm_motor_status[2].position_cdeg<ARM_CLEAR_REACH_CDEG+50,823);'),
('    if(test>=50) return test_standalone(test);','    if(test>=70) return micro_case(test);\n    if(test>=50) return test_standalone(test);'),
('arm_motor_status[0].position_cdeg< -9900','arm_motor_status[0].position_cdeg< -900-ARM_PICK_DROP_CDEG')])
edit('tests/arm_work/run_tests.py',[
('cases = range(1, 43) if mode == 0 else range(50, 60)','cases = list(range(1, 43) if mode == 0 else range(50, 60)) + list(range(70, 83))')], 'utf-8')
for name in ['README_arm.md','README_arm_vision_test.md']:
    p=ROOT/name;s=p.read_text(encoding='utf-8');s+='''

## ALIGN视觉微调（2026-11-19）

ALIGN在保持当前拍照高度（Z目标与反馈差不超过到位容差）、yaw单次变化不超过机构1度、前后单次变化不超过4mm时，直接依次调整yaw和前后轴，跳过容差内已到位的轴，不抬高、不退60mm。全轴已到位则直接报告ALIGNED，命令编号及重复帧规则保留。目标仍为相对原点的绝对电机轴角度，协议无需改变。

配置在arm_config.h：ARM_ALIGN_MICRO_YAW_STEP_CDEG=300（减速3:1，机构1度），ARM_ALIGN_MICRO_REACH_STEP_CDEG=1273（约4mm），ARM_ALIGN_MICRO_MIN_Z_CDEG=ARM_YAW_MIN_Z_CDEG（最高零位向下不超过105mm）。当前及目标前后位置限制在最前起后退0～60mm，避免在靠后120mm端点附近转yaw；超出这些微调条件会走完整避让。微调使用当前反馈计算步幅，并在真正发起yaw/前后动作前再次检查高度、行程和步幅，条件变化报POSE故障。

到工作区拍照、明显改变Z/大角度/大位移ALIGN、Watch yaw测试、FINISH收回仍使用完整避让。ARM_CLEAR_REACH_CDEG=-19098不变。抓取下降宏保留用户设置90000（100mm），不影响微调判据。
''';p.write_text(s,encoding='utf-8')
for mode in [0,1]:
    subprocess.run([sys.executable,'-u','tests/arm_work/run_tests.py','--mode',str(mode)],cwd=ROOT,check=True)
subprocess.run([sys.executable,'tests/arm_work/test_client.py'],cwd=ROOT,check=True)
uv=Path('E:/APP/Keil/Keil5 MDK5.42/UV4/UV4.exe')
xml=(ROOT/'MDK-ARM/car_code1.uvprojx').read_text(encoding='utf-8').replace('<RunUserProg2>1</RunUserProg2>','<RunUserProg2>0</RunUserProg2>')
for mode in [0,1]:
    target=ROOT/f'MDK-ARM/align_micro_verify_{mode}.uvprojx'
    content=xml.replace('<Define>USE_HAL_DRIVER,STM32F103xE</Define>',f'<Define>USE_HAL_DRIVER,STM32F103xE,ARM_VISION_TEST_MODE={mode}</Define>')
    if mode==0:content=content.replace('<OutputDirectory>car_code1\\</OutputDirectory>','<OutputDirectory>..\\tmp\\align_micro_vehicle\\</OutputDirectory>')
    target.write_text(content,encoding='utf-8')
    log=ROOT/f'tmp/align_micro_mode{mode}_build.log'
    try:
        result=subprocess.run([str(uv),'-r',str(target),'-t','car_code1','-o',str(log)],cwd=ROOT,creationflags=subprocess.CREATE_NO_WINDOW)
        report=log.read_text(encoding='gb18030',errors='replace');print(report[-650:],flush=True)
        if '0 Error(s)' not in report:raise RuntimeError(f'Keil build failed mode {mode}: {log}')
    finally:target.unlink(missing_ok=True)
print('Backup:',backup,flush=True)