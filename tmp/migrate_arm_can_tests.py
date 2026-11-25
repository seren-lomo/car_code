from pathlib import Path
r=Path('E:/Inte_handing/car_code1')
def read(n):return (r/n).read_text(encoding='utf8')
def write(n,s):
    with (r/n).open('w',encoding='utf8',newline='\r\n') as f:f.write(s)

write('tests/arm_can/mock.h','''#ifndef ARM_CAN_MOCK_H
#define ARM_CAN_MOCK_H
#define __MAIN_H
#define __CAN_H__
#include <stdint.h>
#define __IO volatile
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t ErrorCode;} CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan;
extern volatile HAL_StatusTypeDef can_last_tx_status;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
void can_SendCmd(volatile uint8_t *,uint8_t);
#endif
''')

for n in ('tests/arm_can/harness.c','tests/arm_work/harness.c'):
    s=read(n).replace('arm_uart.h','arm_can.h').replace('ArmUART','ArmCAN').replace('arm_uart_diag','arm_can_diag').replace('ARM_MOTOR_UART','ARM_MOTOR_CAN')
    s=s.replace('UART_HandleTypeDef huart1={0},huart2={1},huart3={2};','CAN_HandleTypeDef hcan={0};\nvolatile HAL_StatusTypeDef can_last_tx_status=HAL_OK;')
    s=s.replace('static uint8_t *rx[3],response[3][8],response_n[3];','static uint8_t response[3][8],response_n[3];')
    start=s.index('HAL_StatusTypeDef HAL_UART_Receive_IT(')
    end=s.index('    r[0]=f[0];',start)
    s=s[:start]+'''uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *u) {return u->ErrorCode;}
void can_SendCmd(volatile uint8_t *f,uint8_t n)
{
    uint8_t i,*r,function=f[1];
    uint32_t v,pulses;
    tx_count++;
    can_last_tx_status=HAL_ERROR;
    if(tx_fail || f[0]<5 || f[0]>7 || f[n-1]!=0x6B)return;
    i=(uint8_t)(f[0]-5);r=response[i];
'''+s[end:]
    s=s.replace('if(n!=13 || f[10]!=2 || f[11]!=0)return HAL_ERROR;', 'if(n!=13 || f[10]!=2 || f[11]!=0)return;')
    s=s.replace('    ArmCAN_TxCpltCallback(u);\n    return HAL_OK;', '    can_last_tx_status=HAL_OK;')
    start=s.index('static UART_HandleTypeDef *uart(');end=s.index('static void step(',start)
    s=s[:start]+s[end:]
    s=s.replace('uint8_t i,j,n;','uint8_t i,n;')
    s=s.replace('for(j=0;j<n;j++) {*rx[i]=response[i][j];ArmCAN_RxCpltCallback(uart(i));}', 'ArmCAN_RxFrame((uint32_t)(5U+i)<<8,response[i]+1,(uint8_t)(n-1U));')
    if n.endswith('arm_can/harness.c'):
        s=s.replace('''        ArmCAN_ErrorCallback(&huart1);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_CAN && !moving[0] && !arm_motor_status[0].homed,132);''','''        tx_fail=1;step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !arm_motor_status[0].homed,132);''')
        s=s.replace('for(i=0;i<128;i++) {*rx[0]=0x55;ArmCAN_RxCpltCallback(&huart1);}step(500);', '''{uint8_t f[3]={0x3A,1,0x6B};
        for(i=0;i<128;i++) ArmCAN_RxFrame(0x500,f,3);}step(500);''')
        s=s.replace('arm_can_diag[1].rx_bytes==8','arm_can_diag[1].rx_bytes==7')
        s=s.replace('arm_can_diag[1].rx_raw[7]==0x6B','arm_can_diag[1].rx_raw[6]==0x6B')
        s=s.replace('    } else return 999;', '''    } else if(test==18) {
        uint8_t f[7]={0x36,0,0,0,0,42,0x6B};
        CHECK(ArmCAN_Probe(0),180);drop_function=0x36;step(5);
        ArmCAN_RxFrame(0x100,f,7);ArmCAN_RxFrame(0x600,f,7);step(1);
        CHECK(arm_motor_status[0].rx_frames==0 && arm_motor_status[1].rx_frames==1,181);
        ArmCAN_RxFrame(0x500,f,7);step(1);
        CHECK(arm_motor_status[0].state==ARM_MOTOR_DONE && arm_motor_status[0].position_counts==42,182);
    } else if(test==19) {
        uint8_t f[7]={0x36,0,0,0,0,42,0x6B};
        drop_function=0x36;CHECK(ArmCAN_Probe(0),190);step(5);
        ArmCAN_RxFrame(0x501,f,7);ArmCAN_RxFrame(0x500,f,6);
        f[6]=0;ArmCAN_RxFrame(0x500,f,7);step(1);
        CHECK(arm_motor_status[0].rx_frames==0 && arm_motor_status[0].rx_errors==3,191);
        step(100);CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT,192);
    } else if(test==20) {
        arm_motor_status[0].address=1;CHECK(!ArmCAN_Probe(0),200);
        arm_motor_status[0].address=6;CHECK(!ArmCAN_Probe(0),201);
        CHECK(!tx_count,202);
    } else if(test==21) {
        CHECK(home(2),210);CHECK(ArmCAN_Move(2,9000),211);step(4000);
        CHECK(arm_motor_status[2].state==ARM_MOTOR_DONE,212);
        CHECK(arm_can_diag[2].tx_can_frames==arm_can_diag[2].tx_completed+1,213);
    } else return 999;''')
    else:
        s=s.replace('ArmCAN_ErrorCallback(&huart3);step(300);', '''{uint8_t f[3]={0x3A,1,0x6B};for(i=0;i<128;i++) ArmCAN_RxFrame(0x700,f,3);}step(300);''')
    write(n,s)

for n in ('tests/arm_can/run_tests.py','tests/arm_work/run_tests.py'):
    s=read(n).replace('UART driver','CAN driver').replace('arm_uart','arm_can').replace('UART motor','CAN motor')
    if '/arm_can/' in n:s=s.replace('range(1, 18)','range(1, 22)').replace('17/17','21/21')
    write(n,s)
print('Migrated behavioral CPU tests to CAN transport.')
