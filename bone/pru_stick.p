// PRU motor control code
// r29 is call return address

#define PRU_1

.setcallreg r29.w0
.origin 0
.entrypoint START

// Global registers:
// r16-r19 clobbered by macros
// r20 current input state
// r21 current output state
// r22 current idle task
// r23.w0 encoder 0 position
// r23.w2 encoder 1 position
// r24.w0 encoder 2 position
// r24.b2 raise position flags
// r25.b0 last set position 0
// r25.b1 last set position 1
// r25.b2 last set position 2

// A set of [pointer, count] 32-bit pairs
// Zero pointer indicates end of list
#define TASK_LIST 0x100

#define DEBUG_RAM 0xf0

// Communication area offsets
#define COM_SET_POS 0x00
#define COM_RANGE 0x80
#define COM_INPUT_BITS 0x84
#define COM_CURRENT_POS 0x88 // 8 bytes

#define PRU0_ARM_INTERRUPT 19
#define PRU1_ARM_INTERRUPT 20

#define CONST_PRUDRAM C24

#define PRU0_CTRL_BASE 0x22000
#define PRU1_CTRL_BASE 0x24000
#ifdef PRU_0
#define PRU_CTRL_BASE PRU1_CTRL_BASE
#else
#define PRU_CTRL_BASE PRU1_CTRL_BASE
#endif

// Offsets from PRU_CTRL_BASE
#define PRU_CTRL_CONTROL  0x00
#define PRU_CTRL_CYCLE    0x0c

// IO pins
#define BLINK_PIN r30.t0
#define INPUT_CLK_PIN r30.t1
#define INPUT_LATCH_PIN r30.t2
#define INPUT_DATA_PIN r31.t8
#define OUTPUT_CLK_PIN r30.t3
#define OUTPUT_LATCH_PIN r30.t4
#define OUTPUT_DATA_PIN r30.t5

// Clock frequency for input shift registers
// 200/(100 * 2) = 1 MHz
// Actual rate is slower because of loop overhead
#define INPUT_CLK_DELAY 100

#define OUTPUT_CLK_DELAY 100

// Sample inputs every INPUT_PERIOD us
#define INPUT_PERIOD 100 // 10kHz

#define INPUT_NUM_BITS t24

// Input bit numbers
#define ES_0AF t0
#define ES_0AR t1
#define ES_0BF t2
#define ES_0BR t3

#define ES_1AF t8
#define ES_1AR t9
#define ES_1BF t10
#define ES_1BR t11

#define ES_2AF t16
#define ES_2AR t17
#define ES_2BF t18
#define ES_2BR t19

#define ENCODER_STEP 0x40

#define ENCODER0_BITA t4
#define ENCODER0_BITB t5
#define ENCODER1_BITA t12
#define ENCODER1_BITB t13
#define ENCODER2_BITA t20
#define ENCODER2_BITB t21

// MOTOR0A -> terminal 2
// MOTOR0B -> terminal 3
// MOTOR1A -> terminal 1
// MOTOR1B -> terminal 0
// Output bit numbers
// Bit 0 is shifted out first
#define MOTOR0A_HF t0
#define MOTOR0A_HR t1
#define MOTOR0B_HR t2
#define MOTOR0B_HF t3
#define MOTOR0A_LF t4
#define MOTOR0A_LR t5
#define MOTOR0B_LR t6
#define MOTOR0B_LF t7

#define MOTOR1A_HF t8
#define MOTOR1A_HR t9
#define MOTOR1B_HR t10
#define MOTOR1B_HF t11
#define MOTOR1A_LF t12
#define MOTOR1A_LR t13
#define MOTOR1B_LR t14
#define MOTOR1B_LF t15

#define MOTOR2A_HF t16
#define MOTOR2A_HR t17
#define MOTOR2B_HR t18
#define MOTOR2B_HF t19
#define MOTOR2A_LF t20
#define MOTOR2A_LR t21
#define MOTOR2B_LR t22
#define MOTOR2B_LF t23

#define MOTOR_LOW_MASK 0xf0f0f0

.macro delay_loop
.mparam cycles
  mov r16, cycles - 1
delay_loop:
  sub r16, r16, 2
  qbbc delay_loop, r16.t31
.endm

.macro debug
.mparam val
  mov r16, DEBUG_RAM
  mov r17, val
  sbbo r17, r16, 0, 4
.endm

// Schedule a task for execution
.macro add_task
.mparam task, time
  mov r16, TASK_LIST
add_task_loop:
  lbbo r17, r16, 0, 8
  add r16, r16, 8
  qbne add_task_loop, r17, 0
  sub r16, r16, 8
  mov r17, task
  mov r18, time
  sbbo r17, r16, 0, 8
  mov r17, 0
  sbbo r17, r16, 8, 4
.endm

.macro add_task_us
.mparam task, time_us
  add_task task, (time_us*200)
.endm

.macro add_task_ms
.mparam task, time_ms
  add_task_us task, (time_ms*1000)
.endm

// Initialization code
START:
  debug 0
  mov r30, 0
  // Setup task list
  mov r0, 0
  mov r1, TASK_LIST
  sbbo r0, r1, 0, 4

  // Start timer
  mov r0, PRU_CTRL_BASE
  mov r1, 0
  sbbo r1, r0, PRU_CTRL_CYCLE, 4
  lbbo r1, r0, PRU_CTRL_CONTROL, 4
  set r1.t3
  sbbo r1, r0, PRU_CTRL_CONTROL, 4

  // Init globals
  mov r20, 0xffffff
  mov r21, 0
  mov r22, selftest
  mov r23, 0
  mov r24, 0
  sbco r23, CONST_PRUDRAM, COM_RANGE, 4
  mov r0, 0x808080
  sbco r0, CONST_PRUDRAM, COM_SET_POS, 4

  // Schedule initial process
  add_task blink_on, 0
  qba dispatch

// Event dispatch loop
dispatch:
  // Load time elapsed into r2
  mov r0, PRU_CTRL_BASE
  mov r1, 0
  lbbo r2, r0, PRU_CTRL_CYCLE, 4
  sbbo r1, r0, PRU_CTRL_CYCLE, 4
  add r2, r2, 1

  // Update task timers
  mov r3, TASK_LIST
task_sub_loop:
  lbbo r0, r3, 0, 8
  qbeq task_check, r0, 0
  sub r1, r1, r2
  sbbo r1, r3, 4, 4
  add r3, r3, 8
  qba task_sub_loop
task_check:
  // r3 points to the terminator
  // Execute tasks that have expired
  mov r4, TASK_LIST
task_check_loop:
  lbbo r0, r4, 0, 8
  add r4, r4, 8
  // If reached end of list, run idle task
  qbeq idle_start, r0, 0
  // Skip over entries with time remaining
  qbbc task_check_loop, r1.t31
  // Replace this entry with the last one in the list
  sub r3, r3, 8
  sub r4, r4, 8
  lbbo r5, r3, 0, 8
  sbbo r5, r4, 0, 8
  // Then shorten the list
  mov r1, 0
  sbbo r1, r3, 0, 4
  // Jump to the event handler.  This will finish by jumping to dispatch
  // If multiple timers have expired then we catch them in the next iteration
  jmp r0

// Event handlers
blink_on:
  set BLINK_PIN
  add_task_ms blink_off, 500
  qba dispatch

blink_off:
  clr BLINK_PIN
  add_task_ms blink_on, 500
  qba dispatch

idle_start:
  // Sample inputs
  clr INPUT_LATCH_PIN
  delay_loop INPUT_CLK_DELAY
  set INPUT_LATCH_PIN
  mov r0, 1
  mov r1, r20
  mov r20, 0
input_loop:
  qbbc input_zero, INPUT_DATA_PIN
  or r20, r20, r0
input_zero:
  clr INPUT_CLK_PIN
  delay_loop INPUT_CLK_DELAY
  set INPUT_CLK_PIN
  delay_loop INPUT_CLK_DELAY
  lsl r0, r0, 1
  clr r0.INPUT_NUM_BITS
  qbne input_loop, r0, 0

  sbco r20, CONST_PRUDRAM, COM_INPUT_BITS, 4
  // Process quadrate encoders

.macro do_half_quad
.mparam bit0, bit1
  // Expects r2 = changed bits
  // Set:
  //  r3 = 0 if bit0 unchanged
  //  r3 = 1 if bit0 changed and bit0 == bit1
  //  r3 = -1 if bit0 changed and bit0 != bit1
  mov r3, 0
  qbbc half_quad_done, r2.bit0
  mov r3, ENCODER_STEP
  qbbs half_quad_high, r20.bit0
  mov r3, ENCODER_STEP
  qbbc half_quad_done, r20.bit1
  sub r3, r3, 2*ENCODER_STEP
  qba half_quad_done
half_quad_high:
  qbbs half_quad_done, r20.bit1
  sub r3, r3, 2*ENCODER_STEP
half_quad_done:
.endm
.macro do_quad
.mparam val, bit0, bit1
  do_half_quad bit0, bit1
  add val, val, r3
  do_half_quad bit1, bit0
  sub val, val, r3
.endm

  xor r2, r1, r20
  do_quad r23.w0, ENCODER0_BITA, ENCODER0_BITB
  do_quad r23.w2, ENCODER1_BITA, ENCODER1_BITB
  do_quad r24.w0, ENCODER2_BITA, ENCODER2_BITB
  sbco r23.w0.b1, CONST_PRUDRAM, COM_CURRENT_POS, 1
  sbco r23.w2.b1, CONST_PRUDRAM, COM_CURRENT_POS+1, 1
  sbco r24.w0.b1, CONST_PRUDRAM, COM_CURRENT_POS+2, 1

  // Invoke idle task
  jmp r22

idle_end:
  // Set outputs
  mov r0, r21
  mov r1, 24
  clr OUTPUT_LATCH_PIN
output_loop:
  clr OUTPUT_CLK_PIN
  qbbs output_one, r0.t0
  clr OUTPUT_DATA_PIN
  qba output_ready
output_one:
  set OUTPUT_DATA_PIN
output_ready:
  delay_loop OUTPUT_CLK_DELAY
  set OUTPUT_CLK_PIN
  delay_loop OUTPUT_CLK_DELAY
  lsr r0, r0, 1
  sub r1, r1, 1
  qbne output_loop, r1, 0
  set OUTPUT_LATCH_PIN
  qba dispatch

selftest:
  qba calibrate

  // TODO: Maybe calibrate motors one at a time
calibrate:
  mov r21, 0
  set r21.MOTOR0A_HR
  set r21.MOTOR1A_HR
  set r21.MOTOR2A_HR
  set r21.MOTOR0A_LR
  set r21.MOTOR1A_LR
  set r21.MOTOR2A_LR
  mov r22, calibrate_backward

calibrate_backward:
  call check_all_es
  mov r0, MOTOR_LOW_MASK
  and r0, r0, r21
  qbne idle_end, r0, 0

  debug 1
  mov r23.w0, 0
  mov r23.w2, 0
  mov r24.w0, 0

  mov r21, 0
  set r21.MOTOR0A_HF
  set r21.MOTOR1A_HF
  set r21.MOTOR2A_HF
  set r21.MOTOR0A_LF
  set r21.MOTOR1A_LF
  set r21.MOTOR2A_LF
  mov r22, calibrate_forward

calibrate_forward:
  call check_all_es
  mov r0, MOTOR_LOW_MASK
  and r0, r0, r21
  qbne idle_end, r0, 0

  // Now at home position. Calculate encoder range
  lsr r23.w0, r23.w0, 1
  sbco r23.w0.b1, CONST_PRUDRAM, COM_RANGE, 1
  set r23.w0.t15
  lsr r23.w2, r23.w2, 1
  sbco r23.w2.b1, CONST_PRUDRAM, COM_RANGE+2, 1
  set r23.w2.t15
  lsr r24.w0, r24.w0, 1
  sbco r24.w0.b1, CONST_PRUDRAM, COM_RANGE+3, 1
  set r24.w0.t15

  debug 2
  mov r22, run
  qba idle_end

run:
  mov r1, r21
  lbco r0, CONST_PRUDRAM, COM_SET_POS, 4

.macro run_motor_pos
.mparam posbyte, curpos, lf, lr, hf, hr
  qbeq run_pos_stop, r25.posbyte, r0.posbyte
  // TODO: add hysteresis on position detection
  qbgt run_pos_rev, r0.posbyte, curpos
  qblt run_pos_fwd, r0.posbyte, curpos
  mov r25.posbyte, r0.posbyte
run_pos_stop:
  clr r21.hf
  clr r21.hr
  clr r21.lf
  clr r21.lr
  qba run_pos_done
run_pos_rev:
  clr r21.hf
  clr r21.lf
  set r21.hr
  // Delay activation if changing direction
  qbbs run_pos_done, r1.hf
  set r21.lr
  qba run_pos_done
run_pos_fwd:
  clr r21.hr
  clr r21.lr
  set r21.hf
  qbbs run_pos_done, r1.hr
  set r21.lf
  qba run_pos_done
run_pos_done:
  call check_all_es
.endm

.macro run_motor_raise
.mparam bitpos, lf, lr, hf, hr
  qbbs run_raise_done, r21.lf
  qbbs run_raise_done, r21.lr
  qbbs run_raise_stop_f, r21.hf
  qbbs run_raise_stop_r, r21.hr
  qbbs run_raise_f, r24.b3.bitpos
  qbbc run_raise_done, r0.b3.bitpos
  set r21.hr
  set r21.lr
  qba run_raise_done
run_raise_f:
  qbbs run_raise_done, r0.b3.bitpos
  set r21.hf
  set r21.lf
  qba run_raise_done
run_raise_stop_f:
  clr r21.hf
  set r24.b3.bitpos
  qba run_raise_done
run_raise_stop_r:
  clr r21.hr
  clr r24.b3.bitpos
run_raise_done:
.endm

  run_motor_pos b0, r23.w0.b1, MOTOR0A_LF, MOTOR0A_LR, MOTOR0A_HF, MOTOR0A_HR
  run_motor_pos b1, r23.w1.b1, MOTOR1A_LF, MOTOR1A_LR, MOTOR1A_HF, MOTOR1A_HR
  run_motor_pos b2, r23.w2.b1, MOTOR2A_LF, MOTOR2A_LR, MOTOR2A_HF, MOTOR2A_HR
  run_motor_raise t0, MOTOR0B_LF, MOTOR0B_LR, MOTOR0B_HF, MOTOR0B_HR
  run_motor_raise t1, MOTOR1B_LF, MOTOR1B_LR, MOTOR1B_HF, MOTOR1B_HR
  run_motor_raise t2, MOTOR2B_LF, MOTOR2B_LR, MOTOR2B_HF, MOTOR2B_HR

  qba idle_end


.macro check_es
.mparam es, motor
  qbbc check_es_skip, r20.es
  clr r21.motor
check_es_skip:
.endm

check_all_es:
  check_es ES_0AF, MOTOR0A_LF
  check_es ES_0BF, MOTOR0B_LF
  check_es ES_1AF, MOTOR1A_LF
  check_es ES_1BF, MOTOR1B_LF
  check_es ES_2AF, MOTOR2A_LF
  check_es ES_2BF, MOTOR2B_LF
  check_es ES_0AR, MOTOR0A_LR
  check_es ES_0BR, MOTOR0B_LR
  check_es ES_1AR, MOTOR1A_LR
  check_es ES_1BR, MOTOR1B_LR
  check_es ES_2AR, MOTOR2A_LR
  check_es ES_2BR, MOTOR2B_LR
  ret


interrupt:
  MOV R31.b0, PRU1_ARM_INTERRUPT+16   // Send notification to Host for program completion
  qba dispatch
