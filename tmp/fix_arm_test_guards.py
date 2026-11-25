from pathlib import Path

root = Path(__file__).resolve().parents[1]

def edit(name, changes):
    path = root / name
    raw = path.read_bytes()
    newline = b'\r\n' if b'\r\n' in raw else b'\n'
    text = raw.replace(b'\r\n', b'\n').decode('gb18030')
    for before, after in changes:
        assert before in text, (name, before[:100])
        text = text.replace(before, after, 1)
    path.write_bytes(text.encode('gb18030').replace(b'\n', newline))

edit('Core/Src/route.c', [
('''    if(ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
    if(area''', '''#if ARM_VISION_TEST_MODE
    (void)area;return ROUTE_BAD_STATE;
#else
    if(area'''),
('''    send_start_area();
    return ROUTE_OK;
}''', '''    send_start_area();
    return ROUTE_OK;
#endif
}'''),
('''RouteResult_t Route_Start(uint32_t route_id)
{
''', '''RouteResult_t Route_Start(uint32_t route_id)
{
#if ARM_VISION_TEST_MODE
    (void)route_id;return ROUTE_BAD_STATE;
#else
'''),
('''    if(ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
    if(route_id''', '''    if(route_id'''),
])
# Close the vehicle implementation at the matching function brace.
path = root / 'Core/Src/route.c'
raw = path.read_bytes()
start = raw.index(b'RouteResult_t Route_Start(')
end = raw.index(b'\n}\n', start)
raw = raw[:end] + b'\n#endif' + raw[end:]
path.write_bytes(raw)

edit('Core/Src/arm_work.c', [
('''    if(ARM_VISION_TEST_MODE) return !Route_IsActive() && !Chassis_IsBusy();
    return route_status.state==ROUTE_WAIT_WORK && id==route_status.route_id &&
        index==route_status.point_index && work==route_status.work_id;''', '''#if ARM_VISION_TEST_MODE
    return !Route_IsActive() && !Chassis_IsBusy();
#else
    return route_status.state==ROUTE_WAIT_WORK && id==route_status.route_id &&
        index==route_status.point_index && work==route_status.work_id;
#endif'''),
('''        if(!ARM_VISION_TEST_MODE) return ROUTE_BAD_STATE;
        if(!id''', '''#if !ARM_VISION_TEST_MODE
        return ROUTE_BAD_STATE;
#else
        if(!id'''),
('''        remember(p);report();return ROUTE_OK;
    }
    /* Validate identity''', '''        remember(p);report();return ROUTE_OK;
#endif
    }
    /* Validate identity'''),
])
