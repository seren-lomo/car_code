from pathlib import Path

r=Path('E:/Inte_handing/car_code1')
def read(name):return (r/name).read_bytes().replace(b'\r\n',b'\n')
def write(name,b): (r/name).write_bytes(b.replace(b'\n',b'\r\n'))
def replace(b,old,new):
    old=old.encode();new=new.encode()
    assert b.count(old)==1,(old[:90],b.count(old))
    return b.replace(old,new)

b=read('Core/Src/arm_uart.c').replace(b'arm_uart.h',b'arm_can.h').replace(b'usart.h',b'can.h')
b=b.replace(b'ArmUART',b'ArmCAN').replace(b'arm_uart_diag',b'arm_can_diag').replace(b'ARM_MOTOR_UART',b'ARM_MOTOR_CAN')
b=replace(b,'''    UART_HandleTypeDef *uart;
    uint8_t byte, rx[RX_SIZE], frame[8], tx[13];''','''    uint8_t rx[RX_SIZE], frame[8], tx[13];''')
b=replace(b,'    uint8_t used, waiting, expected, reply, tx_active, rx_restart;', '    uint8_t used, waiting, expected, reply, tx_active;')
start=b.index(b'static uint8_t index_of(');end=b.index(b'static void fail(',start)
b=b[:start]+b[end:]
start=b.index(b'    result=HAL_UART_Transmit_IT(');end=b.index(b'    if(result!=HAL_OK)',start)
b=b[:start]+b'''    /* Shared foreground transport waits for actual CAN transmission. All
     * callers (chassis and arm) belong to RobotTask; no ISR/CommTask sends here.
     * Position commands are split by can_SendCmd into consecutive packets.
     */
    can_SendCmd(p->tx,n);
    result=can_last_tx_status;
    arm_can_diag[i].last_hal_status=(uint32_t)result;
    arm_can_diag[i].last_can_error=HAL_CAN_GetError(&hcan);
    p->tx_active=0U;
    if(result==HAL_OK) {
        arm_can_diag[i].tx_completed++;
        arm_can_diag[i].tx_can_frames+=(uint32_t)((n-2U+6U)/7U);
    }
'''+b[end:]
start=b.index(b'HAL_StatusTypeDef ArmCAN_Init(');end=b.index(b'\nstatic void accept(',start)
b=b[:start]+b'''HAL_StatusTypeDef ArmCAN_Init(void)
{
    uint8_t i;
    for(i=0;i<ARM_AXIS_COUNT;i++) {
        Port empty={0}; ArmMotorStatus_t status={0}; ArmCANDiag_t diag={0};
        ports[i]=empty; arm_motor_status[i]=status; arm_can_diag[i]=diag;
        arm_motor_status[i].address=(uint8_t)(5U+i);
    }
    return HAL_OK; /* CAN/filter/FIFO IRQ already started by main. No motion. */
}

/* Called only by the shared CAN FIFO0 ISR. Current replies fit one CAN frame.
 * Preserve whole validated frame boundaries before reconstructing the common
 * address/function/data/6B representation consumed in foreground.
 */
void ArmCAN_RxFrame(uint32_t ext_id,const uint8_t *data,uint8_t dlc)
{
    uint8_t i,j,expected,n;
    uint16_t head,free_bytes;
    uint32_t address=ext_id>>8;
    Port *p;
    if(address<5U || address>7U) return;
    for(i=0U;i<ARM_AXIS_COUNT;i++) if(arm_motor_status[i].address==address) break;
    if(i==ARM_AXIS_COUNT) return;
    p=&ports[i];
    arm_can_diag[i].rx_can_frames++;
    arm_can_diag[i].last_rx_ext_id=ext_id;
    arm_can_diag[i].last_rx_dlc=dlc;
    if(!data || dlc<2U || dlc>8U) {arm_motor_status[i].rx_errors++;return;}
    arm_can_diag[i].rx_bytes+=dlc;
    for(j=0U;j<dlc && arm_can_diag[i].rx_length<16U;j++) {
        uint8_t slot=arm_can_diag[i].rx_length;
        arm_can_diag[i].rx_raw[slot]=data[j];
        arm_can_diag[i].rx_length=(uint8_t)(slot+1U);
    }
    if(data[0]==0x36U) expected=7U;
    else if(data[0]==0x35U) expected=5U;
    else if(data[0]==0x3AU || data[0]==0xF3U || data[0]==0xFDU || data[0]==0xFEU) expected=3U;
    else return; /* e.g. chassis broadcast-sync reply; not an arm operation. */
    if((ext_id&0xFFU)!=0U || dlc!=expected || data[dlc-1U]!=0x6BU) {
        arm_motor_status[i].rx_errors++;return;
    }
    n=(uint8_t)(dlc+1U);
    free_bytes=(uint16_t)((p->tail-p->head-1U)&(RX_SIZE-1U));
    if(free_bytes<n) {p->rx_fault=1U;return;}
    head=p->head;p->rx[head]=(uint8_t)address;head=(uint16_t)((head+1U)&(RX_SIZE-1U));
    for(j=0U;j<dlc;j++) {p->rx[head]=data[j];head=(uint16_t)((head+1U)&(RX_SIZE-1U));}
    __DMB();p->head=head;
}
'''+b[end:]
b=replace(b,'''    arm_motor_status[i].rx_frames++;
    /* Late replies''','''    if((function==0x36U || function==0x35U) && f[2]>1U) {
        arm_motor_status[i].rx_errors++;return;
    }
    arm_motor_status[i].rx_frames++;
    /* Late replies''')
b=replace(b,'    if(p->rx_fault || p->rx_restart || p->tx_active) return 0U;', '''    /* On a shared bus these addresses must never overlap chassis 1..4. */
    if(arm_motor_status[i].address<5U || arm_motor_status[i].address>7U) return 0U;
    {uint8_t j;for(j=0U;j<ARM_AXIS_COUNT;j++)
        if(j!=i && arm_motor_status[j].address==arm_motor_status[i].address) return 0U;}
    if(p->rx_fault || p->tx_active) return 0U;''')
b=replace(b,'        if(p->tx_active) HAL_UART_AbortTransmit(p->uart);\n        p->tx_active=p->waiting=p->reply=p->operation=0U;','        /* CAN sends finish before returning; no UART/DMA owns this axis. */\n        p->tx_active=p->waiting=p->reply=p->operation=0U;')
start=b.index(b'        if(p->rx_restart) {');end=b.index(b'        parse(i);',start)
b=b[:start]+b[end:]
b=replace(b,'            if(p->tx_active) HAL_UART_AbortTransmit(p->uart);','            /* The CAN transfer is already complete; only reply may time out. */')
b=b.replace(b'cause==1U?ARM_MOTOR_RX_OVERFLOW:ARM_MOTOR_CAN',b'cause==1U?ARM_MOTOR_RX_OVERFLOW:ARM_MOTOR_CAN')
write('Core/Src/arm_can.c',b)

b=read('Core/Inc/arm_uart.h').replace(b'CAR_ARM_UART_H',b'CAR_ARM_CAN_H').replace(b'ArmUART',b'ArmCAN').replace(b'arm_uart_diag',b'arm_can_diag').replace(b'ARM_MOTOR_UART',b'ARM_MOTOR_CAN')
b=b.replace(b'state, fault, tick, rx_frames, rx_errors, uart_errors;',b'state, fault, tick, rx_frames, rx_errors, can_errors;')
b=replace(b,' * 16 received bytes of the most recently attempted TX, including rejected bytes.', ' * 16 CAN payload bytes received since the most recently attempted TX.\n * tx_raw retains the logical address/function/data/6B command before CAN splitting.')
b=replace(b,'    uint8_t tx_length, tx_raw[13], rx_length, rx_raw[16];','''    uint32_t tx_can_frames, rx_can_frames, last_rx_ext_id, last_rx_dlc, last_can_error;
    uint8_t tx_length, tx_raw[13], rx_length, rx_raw[16];''')
b=replace(b,'''/* Init only arms UART RX; it never enables or moves a motor. Task and command
 * functions have one owner: RobotTask. RX callbacks only enqueue bytes.
 */''','''/* Shared CAN1, chassis IDs 1..4; Z/yaw/reach IDs 5/6/7. Init resets local
 * state only; main owns CAN startup. Task/commands: RobotTask only. The shared
 * FIFO0 ISR calls RxFrame, which only enqueues complete single-frame replies.
 */''')
start=b.index(b'void ArmCAN_RxCpltCallback(');end=b.index(b'\n\n#endif',start)
b=b[:start]+b'void ArmCAN_RxFrame(uint32_t ext_id,const uint8_t *data,uint8_t dlc);'+b[end:]
write('Core/Inc/arm_can.h',b)

# Leave a source-level include shim; the old transport is no longer built.
write('Core/Inc/arm_uart.h',b'''#ifndef CAR_ARM_UART_COMPAT_H
#define CAR_ARM_UART_COMPAT_H
/* Legacy include only. Motors now use CAN: see arm_can.h and arm_can_diag. */
#include "arm_can.h"
#endif
''')
write('Core/Src/arm_uart.c',b'''/* Retired UART transport. The Keil project builds arm_can.c instead.
 * This file intentionally defines no driver or UART callbacks.
 */
''')
for name in ('Core/Src/arm_work.c','Core/Inc/arm_work.h'):
    b=read(name).replace(b'arm_uart.h',b'arm_can.h').replace(b'ArmUART',b'ArmCAN')
    write(name,b)
b=read('Core/Src/can.c')
b=replace(b,'#include "can.h"','#include "can.h"\n#include "arm_can.h"')
b=replace(b,'volatile MotorFeedback_t motor_feedback[5];','volatile MotorFeedback_t motor_feedback[8];')
b=replace(b,'''        if ((header.ExtId & 0xFFU) != 0U || (header.ExtId >> 8) > 4U)''','''        ArmCAN_RxFrame(header.ExtId, data, (uint8_t)header.DLC);
        if ((header.ExtId & 0xFFU) != 0U || (header.ExtId >> 8) > 7U)''')
write('Core/Src/can.c',b)
b=read('Core/Inc/can.h').replace(b'CAN address 1..4',b'CAN address 1..7').replace(b'motor_feedback[5]',b'motor_feedback[8]')
write('Core/Inc/can.h',b)
b=read('Core/Src/usart.c').replace(b'#include "arm_uart.h"\n',b'')
for line in (b'  ArmUART_RxCpltCallback(huart);\n',b'  ArmUART_ErrorCallback(huart);\n',b'  ArmUART_TxCpltCallback(huart);\n'):
    assert line in b;b=b.replace(line,b'')
write('Core/Src/usart.c',b)
b=read('Core/Src/freertos.c').replace(b'Independent USART1/2/3 motor receive and position state machines.',b'Shared CAN1 arm IDs 5/6/7; same RobotTask owns all CAN sends.')
write('Core/Src/freertos.c',b)
b=read('MDK-ARM/car_code1.uvprojx').replace(b'arm_uart.c',b'arm_can.c').replace(b'arm_uart.h',b'arm_can.h')
write('MDK-ARM/car_code1.uvprojx',b)
print('Migrated arm driver and shared CAN dispatch; IDs 1..7 accepted.')
