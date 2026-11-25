from pathlib import Path

root = Path('E:/Inte_handing/car_code1')
def edit(name, old, new):
    p = root / name
    b = p.read_bytes()
    nl = b'\r\n' if b'\r\n' in b else b'\n'
    old = old.encode().replace(b'\n', nl)
    new = new.encode().replace(b'\n', nl)
    assert b.count(old) == 1, (name, old[:80], b.count(old))
    p.write_bytes(b.replace(old, new))

edit('Core/Inc/arm_uart.h', 'extern volatile ArmMotorConfig_t', '''/* Diagnostic counters are cumulative since reset. rx_raw holds the first
 * 16 received bytes of the most recently attempted TX, including rejected bytes.
 * timeout_* freezes the last timeout before stop commands overwrite live state.
 */
typedef struct {
    uint32_t tx_requests, tx_completed, rx_bytes, last_hal_status;
    uint32_t timeout_function, timeout_stage, timeout_tx_active, timeout_waiting;
    uint8_t tx_length, tx_raw[13], rx_length, rx_raw[16];
} ArmUARTDiag_t;
extern volatile ArmUARTDiag_t arm_uart_diag[ARM_AXIS_COUNT];

extern volatile ArmMotorConfig_t''')
edit('Core/Inc/arm_uart.h', 'uint8_t ArmUART_RecordOrigin', 'uint8_t ArmUART_Probe(uint8_t axis); /* one 0x36 read; no enable/move/zero */\nuint8_t ArmUART_RecordOrigin')
edit('Core/Src/arm_uart.c', 'volatile ArmMotorStatus_t arm_motor_status[ARM_AXIS_COUNT];', 'volatile ArmMotorStatus_t arm_motor_status[ARM_AXIS_COUNT];\nvolatile ArmUARTDiag_t arm_uart_diag[ARM_AXIS_COUNT];')
edit('Core/Src/arm_uart.c', '''    arm_motor_status[i].fault=fault;
    arm_motor_status[i].state=ARM_MOTOR_FAULT;
    arm_motor_status[i].homed=0U;
    p->operation=p->waiting=p->reply=0U;
    p->stop_pending=1U; p->stop_attempts=0U;''', '''    uint8_t probe=(uint8_t)(p->operation==3U);
    if(fault==ARM_MOTOR_TIMEOUT) {
        arm_uart_diag[i].timeout_function=p->expected;
        arm_uart_diag[i].timeout_stage=p->stage;
        arm_uart_diag[i].timeout_tx_active=p->tx_active;
        arm_uart_diag[i].timeout_waiting=p->waiting;
    }
    arm_motor_status[i].fault=fault;
    arm_motor_status[i].state=ARM_MOTOR_FAULT;
    if(!probe) arm_motor_status[i].homed=0U;
    p->operation=p->waiting=p->reply=0U;
    /* A read-only probe must not send an enable, stop, or motion command. */
    if(!probe) {p->stop_pending=1U; p->stop_attempts=0U;}''')
edit('Core/Src/arm_uart.c', '''    Port *p=&ports[i];
    if(p->tx_active) return 0U;
    clear_rx(p);''', '''    Port *p=&ports[i];
    HAL_StatusTypeDef result;
    uint8_t j;
    uint32_t key;
    if(p->tx_active) return 0U;
    clear_rx(p);''')
edit('Core/Src/arm_uart.c', '''    if(HAL_UART_Transmit_IT(p->uart,p->tx,n)!=HAL_OK) {''', '''    arm_uart_diag[i].tx_requests++;
    arm_uart_diag[i].tx_length=n;
    for(j=0U;j<n;j++) arm_uart_diag[i].tx_raw[j]=p->tx[j];
    key=__get_PRIMASK(); __disable_irq();
    arm_uart_diag[i].rx_length=0U;
    for(j=0U;j<16U;j++) arm_uart_diag[i].rx_raw[j]=0U;
    __set_PRIMASK(key);
    result=HAL_UART_Transmit_IT(p->uart,p->tx,n);
    arm_uart_diag[i].last_hal_status=(uint32_t)result;
    if(result!=HAL_OK) {''')
edit('Core/Src/arm_uart.c', '''        Port empty={0}; ArmMotorStatus_t status={0};
        ports[i]=empty; arm_motor_status[i]=status;''', '''        Port empty={0}; ArmMotorStatus_t status={0}; ArmUARTDiag_t diag={0};
        ports[i]=empty; arm_motor_status[i]=status; arm_uart_diag[i]=diag;''')
edit('Core/Src/arm_uart.c', '''    p=&ports[i]; next=(uint16_t)((p->head+1U)&(RX_SIZE-1U));''', '''    p=&ports[i]; next=(uint16_t)((p->head+1U)&(RX_SIZE-1U));
    arm_uart_diag[i].rx_bytes++;
    if(arm_uart_diag[i].rx_length<16U) {
        uint8_t slot=arm_uart_diag[i].rx_length;
        arm_uart_diag[i].rx_raw[slot]=p->byte;
        arm_uart_diag[i].rx_length=(uint8_t)(slot+1U);
    }''')
edit('Core/Src/arm_uart.c', '{ uint8_t i=index_of(u); if(i<ARM_AXIS_COUNT) ports[i].tx_done=1U; }', '''{
    uint8_t i=index_of(u);
    if(i<ARM_AXIS_COUNT) {arm_uart_diag[i].tx_completed++; ports[i].tx_done=1U;}
}''')
edit('Core/Src/arm_uart.c', 'uint8_t ArmUART_RecordOrigin(uint8_t i)', '''uint8_t ArmUART_Probe(uint8_t i)
{ return begin(i,3U); }
uint8_t ArmUART_RecordOrigin(uint8_t i)''')
edit('Core/Src/arm_uart.c', '''    if(p->stage==0U) {
        p->tx[2]=0xABU;''', '''    if(p->operation==3U) {
        if(p->stage==0U) {if(query(i,0x36U)) p->stage=1U;}
        else if(p->reply) {arm_motor_status[i].state=ARM_MOTOR_DONE; p->operation=0U;}
        return;
    }
    if(p->stage==0U) {
        p->tx[2]=0xABU;''')
edit('Core/Src/arm_uart.c', '''            if(p->tx_active) HAL_UART_AbortTransmit(p->uart);
            p->tx_active=p->waiting=0U;
            if(p->operation) fail(i,ARM_MOTOR_TIMEOUT);''', '''            if(p->operation) fail(i,ARM_MOTOR_TIMEOUT);
            if(p->tx_active) HAL_UART_AbortTransmit(p->uart);
            p->tx_active=p->waiting=0U;''')
edit('Core/Src/arm_work.c', '''        } else arm_work_debug.result=ROUTE_BAD_VALUE;
    }
    if(managed''', '''        } else if(arm_work_debug.command==4U) {
            if(arm_work_debug.axis>=ARM_AXIS_COUNT) arm_work_debug.result=ROUTE_BAD_VALUE;
            else if(Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy())
                arm_work_debug.result=ROUTE_BUSY;
            else arm_work_debug.result=ArmUART_Probe((uint8_t)arm_work_debug.axis)?ROUTE_OK:ROUTE_BUSY;
        } else arm_work_debug.result=ROUTE_BAD_VALUE;
    }
    if(managed''')
edit('Core/Inc/arm_work.h', ' * Tests are rejected during routes/work.', ' * 4 read position of selected axis only (works before origins / after fault).\n * Probe completion is motor state 2; it does not record origins or clear work fault.\n * Tests are rejected during routes/work.')

edit('tests/arm_uart/harness.c', '    } else return 999;', '''    } else if(test==15) {
        CHECK(ArmUART_Probe(1),150);step(100);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE && !arm_motor_status[1].homed,151);
        CHECK(tx_count==1 && !stop_count && !move_count && arm_motor_status[1].position_counts==2000,152);
        CHECK(arm_uart_diag[1].tx_completed==1 && arm_uart_diag[1].rx_bytes==8,153);
        CHECK(arm_uart_diag[1].tx_raw[1]==0x36 && arm_uart_diag[1].rx_raw[7]==0x6B,154);
    } else if(test==16) {
        drop_function=0x36;CHECK(ArmUART_Probe(0),160);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT && !ArmUART_Busy(),161);
        CHECK(tx_count==1 && !stop_count && !move_count && !arm_uart_diag[0].rx_bytes,162);
        CHECK(arm_uart_diag[0].timeout_function==0x36 && !arm_uart_diag[0].timeout_tx_active && arm_uart_diag[0].timeout_waiting,163);
        drop_function=0;CHECK(ArmUART_Probe(1),164);step(100);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE,165);
    } else if(test==17) {
        CHECK(!ArmUART_Probe(3),170);CHECK(ArmUART_Probe(0),171);
        CHECK(!ArmUART_Probe(1),172);tx_fail=1;step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !stop_count && !move_count && !ArmUART_Busy(),173);
    } else return 999;''')
edit('tests/arm_uart/run_tests.py', 'range(1, 15)', 'range(1, 18)')
edit('tests/arm_uart/run_tests.py', '14/14 UART', '17/17 UART')
edit('tests/arm_work/harness.c', '    CHECK(home(),2);', '''    if(test==28 || test==29 || test==30) {
        if(test==28) {drop_function=0xF3;CHECK(command(1,1,0,0,0,0)==ROUTE_OK,280);step(500);CHECK(arm_work_status.state==ARM_WORK_FAULT,281);drop_function=0;}
        if(test==29) route_status.state=ROUTE_RUNNING;
        arm_work_debug.command=4;arm_work_debug.axis=test==30?3:1;arm_work_debug.trigger++;
        before=tx_count;step(100);
        if(test==29 || test==30) {CHECK(arm_work_debug.result==(test==29?ROUTE_BUSY:ROUTE_BAD_VALUE) && tx_count==before,290);return 0;}
        CHECK(arm_work_debug.result==ROUTE_OK && arm_motor_status[1].state==ARM_MOTOR_DONE,282);
        CHECK(tx_count==before+1 && !move_count && !arm_motor_status[1].homed,283);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && Route_CheckStart()==ROUTE_BAD_STATE,284);return 0;
    }
    CHECK(home(),2);''')
edit('tests/arm_work/run_tests.py', 'range(1, 28)', 'range(1, 31)')
edit('tests/arm_work/run_tests.py', '27/27 manipulator', '30/30 manipulator')
print('Applied UART diagnostics and independent read-only probe.')
