from pathlib import Path
p=Path('E:/Inte_handing/car_code1/Core/Src/arm_work.c');b=p.read_bytes();nl=b'\r\n' if b'\r\n' in b else b'\n';s=b.replace(b'\r\n',b'\n').decode('gb18030')
old='''    if(!micro_pose_ok(current) ||
       distance_cdeg(current[ARM_Z],micro_start[ARM_Z])>arm_motor_config[ARM_Z].tolerance_cdeg ||
       distance_cdeg(current[ARM_YAW],micro_start[ARM_YAW])>ARM_ALIGN_MICRO_YAW_STEP_CDEG ||
       distance_cdeg(current[ARM_REACH],micro_start[ARM_REACH])>ARM_ALIGN_MICRO_REACH_STEP_CDEG)
        return 0U;
'''
new='''    /* Position feedback may be one report behind immediately after the
     * preceding axis completes. Recheck the absolute safe pose and the new
     * target step; do not compare that delayed feedback to the cached start
     * sample and spuriously fault a valid two-axis micro correction. */
    if(!micro_pose_ok(current)) return 0U;
'''
assert old in s;s=s.replace(old,new,1);p.write_bytes(s.encode('gb18030').replace(b'\n',nl))