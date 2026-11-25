from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def edit(name, changes):
    path = ROOT / name
    raw = path.read_bytes()
    newline = b'\r\n' if b'\r\n' in raw else b'\n'
    text = raw.replace(b'\r\n', b'\n').decode('gb18030')
    for before, after in changes:
        assert before in text, (name, before[:100])
        text = text.replace(before, after, 1)
    path.write_bytes(text.encode('gb18030').replace(b'\n', newline))

edit('Core/Inc/arm_config.h', [('#define CAR_ARM_CONFIG_H\n', '''#define CAR_ARM_CONFIG_H

/* 0: complete vehicle; 1: stationary UART4 vision/arm commissioning.
 * Rebuild and download after changing. Test mode blocks route START and
 * chassis Watch moves; selection keys only record the manual arm origins.
 */
#ifndef ARM_VISION_TEST_MODE
#define ARM_VISION_TEST_MODE      1
#endif
#if ARM_VISION_TEST_MODE != 0 && ARM_VISION_TEST_MODE != 1
#error ARM_VISION_TEST_MODE_must_be_0_or_1
#endif
''')])
edit('Core/Inc/route_keys.h', [('    uint8_t both_pressed;', '    uint32_t arm_test_ready_count; /* origins recorded without route selection */\n    uint8_t both_pressed;')])
edit('Core/Src/route_keys.c', [('#include "route.h"', '#include "route.h"\n#include "arm_config.h"'),
('''        if(result==ROUTE_OK) result=Route_RequestStartArea((uint8_t)route_keys_debug.pending_area);
        route_keys_debug.pose_result=result;
        if(result==ROUTE_OK) route_keys_debug.area_requests++;''', '''#if ARM_VISION_TEST_MODE
        if(result==ROUTE_OK) route_keys_debug.arm_test_ready_count++;
#else
        if(result==ROUTE_OK) result=Route_RequestStartArea((uint8_t)route_keys_debug.pending_area);
        if(result==ROUTE_OK) route_keys_debug.area_requests++;
#endif
        route_keys_debug.pose_result=result;
        if(result==ROUTE_OK) { /* counted above for the selected build mode */ }''')])
edit('Core/Src/route.c', [('#include "route.h"', '#include "route.h"\n#include "arm_config.h"'),
('''RouteResult_t Route_RequestStartArea(uint8_t area)
{
''', '''RouteResult_t Route_RequestStartArea(uint8_t area)
{
    if(ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
'''),
('''    uint16_t i;
    if(route_id!=route_status.route_id''', '''    uint16_t i;
    if(ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
    if(route_id!=route_status.route_id''')])
edit('Core/Src/chassis_debug.c', [('#include "chassis_debug.h"', '#include "chassis_debug.h"\n#include "arm_config.h"'),
('''    ++chassis_task_count;
''', '''    if(ARM_VISION_TEST_MODE) chassis_debug.allow_manual_tests=0U;
    ++chassis_task_count;
''')])
edit('Core/Src/arm_work.c', [
('#define CMD_FINISH 5U', '#define CMD_FINISH 5U\n#define CMD_TEST_BEGIN 6U'),
('static uint32_t highest_command, debug_trigger;', 'static uint32_t highest_command, debug_trigger, last_test_session;'),
('    flags|=4U; put16(out+28,flags);', '    flags|=4U;\n    if(ARM_VISION_TEST_MODE) flags|=8U;\n    put16(out+28,flags);'),
('    if(was_managed && route_status.state==ROUTE_WAIT_WORK)', '    if(!ARM_VISION_TEST_MODE && was_managed && route_status.state==ROUTE_WAIT_WORK)'),
('    highest_command=debug_trigger=event_sequence=0U;', '    highest_command=debug_trigger=event_sequence=last_test_session=0U;'),
('void Route_OnWork(uint32_t id,uint16_t index,uint16_t work)', 'static void begin_work(uint32_t id,uint16_t index,uint16_t work)'),
('''void Route_OnStop(void)
''', '''void Route_OnWork(uint32_t id,uint16_t index,uint16_t work)
{
    if(!ARM_VISION_TEST_MODE) begin_work(id,index,work);
}
static uint8_t context_valid(uint32_t id,uint16_t index,uint16_t work)
{
    if(!managed || id!=arm_work_status.route_id || index!=arm_work_status.point_index ||
       work!=arm_work_status.work_id) return 0U;
    if(ARM_VISION_TEST_MODE) return !Route_IsActive() && !Chassis_IsBusy();
    return route_status.state==ROUTE_WAIT_WORK && id==route_status.route_id &&
        index==route_status.point_index && work==route_status.work_id;
}
void Route_OnStop(void)
'''),
('if(opcode<CMD_ZERO || opcode>CMD_FINISH)', 'if(opcode<CMD_ZERO || opcode>CMD_TEST_BEGIN)'),
('''    /* Validate identity before matching duplicate commands.''', '''    if(opcode==CMD_TEST_BEGIN) {
        /* New opcode is deliberately unavailable in the complete-vehicle build.
         * route_id is a monotonic nonzero test session, index=0, command=1. */
        if(!ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
        if(!id || index || command!=1U || !work_known(work)) return ROUTE_BAD_VALUE;
        if(id<=last_test_session) {
            if(id==last_test_session && context_valid(id,index,work)) {report();return ROUTE_OK;}
            return ROUTE_CONFLICT;
        }
        if(Route_IsActive() || Chassis_IsBusy() || ArmCAN_Busy() || plan_count)
            return ROUTE_BUSY;
        if(!origins() || (arm_work_status.state!=ARM_WORK_READY &&
           arm_work_status.state!=ARM_WORK_READY_LEAVE)) return ROUTE_BAD_STATE;
        last_test_session=id;
        begin_work(id,index,work);
        remember(p);report();return ROUTE_OK;
    }
    /* Validate identity before matching duplicate commands.'''),
('''        if(!managed || route_status.state!=ROUTE_WAIT_WORK || id!=route_status.route_id ||
           id!=arm_work_status.route_id || index!=arm_work_status.point_index ||
           index!=route_status.point_index || work!=arm_work_status.work_id || work!=route_status.work_id)
''', '''        if(!context_valid(id,index,work))
'''),
('''    if(managed && (route_status.state!=ROUTE_WAIT_WORK ||
       route_status.route_id!=arm_work_status.route_id || route_status.point_index!=arm_work_status.point_index)) {''', '''    if(managed && !context_valid(arm_work_status.route_id,arm_work_status.point_index,
       arm_work_status.work_id)) {''')])
