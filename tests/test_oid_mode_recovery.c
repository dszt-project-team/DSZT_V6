/* Regression: the real OID parser/scheduler uses a fresh, locked low-speed
 * mode readback to enter the existing zero/rearm path. No hardware access.
 * Injecting a driver mode change is NOT proof of the field incident cause. */
#define main oid_regression_main
#include "test_oid_diagnostics.c"
#undef main

static void setup_stopped(FrontDrive_Handle_t *d, BSP_RS485_Bus_t *b)
{
  init_ready(d,b);
  target[0]=target[1]=0;
  d->left.status.speed_erpm=d->right.status.speed_erpm=0;
  d->control_read_enabled=1;
  d->control_read_allowed=1;
}

static uint16_t make_mode_reply(uint8_t *f, uint16_t m)
{
  memset(f,0,16);
  f[0]=1; f[1]=3; f[2]=8;
  f[3]=(uint8_t)(m>>8); f[4]=(uint8_t)m;
  return crc(f,11);
}

static void recover_mismatch(uint8_t side, uint16_t mismatched_mode)
{
  FrontDrive_Handle_t d;
  BSP_RS485_Bus_t b;
  uint32_t requested;
  setup_stopped(&d,&b);
  mode[side]=mismatched_mode;
  /* Retain an unsent nonzero target: detecting mismatch must discard it. */
  d.pending_left_erpm=800; d.pending_right_erpm=800;
  for(tick=101;tick<2001;tick++)
  {
    FrontDrive_Task(&d,tick);
    assert(target[0]==0 && target[1]==0);
  }
  assert(d.safety_state==FRONT_DRIVE_SAFETY_WAIT_REARM);
  assert(!d.pending_left_erpm && !d.pending_right_erpm);
  assert(!FrontDrive_IsMotionReady(&d,tick));
  assert(FrontDrive_SetTargetErpm(&d,800,800)==HAL_BUSY);
  assert(mode[0]==OID_ESC_MODE_SPEED && mode[1]==OID_ESC_MODE_SPEED);
  assert(hb_count[0]>10 && hb_count[1]>10);
  assert(max_hb_gap[0]<200 && max_hb_gap[1]<200);
  requested=tick;
  /* This explicit call represents the application's RC-online + centered
     permission. Waiting alone above never released the guard. */
  for(;tick<requested+1000;tick++)
  {
    FrontDrive_Task(&d,tick);
    FrontDrive_RearmSafety(&d,tick);
    assert(target[0]==0 && target[1]==0);
  }
  assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  assert(FrontDrive_IsMotionReady(&d,tick));
  d.control_read_allowed=0;
  assert(FrontDrive_SetTargetErpm(&d,400,400)==HAL_OK);
  requested=tick;
  for(;tick<requested+300;tick++) FrontDrive_Task(&d,tick);
  assert(target[0]==400 && target[1]==400);
}

int main(void)
{
  FrontDrive_Handle_t d;
  BSP_RS485_Bus_t b;
  uint8_t f[16];
  uint16_t n;
  uint32_t reads;

  recover_mismatch(0,OID_ESC_MODE_CURRENT);
  recover_mismatch(1,OID_ESC_MODE_DUTY);

  /* Normal mode readbacks do not repeatedly enter the recovery sequence. */
  setup_stopped(&d,&b);
  for(tick=101;tick<2101;tick++)
  {
    FrontDrive_Task(&d,tick);
    assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  }
  assert(d.left.diagnostic.control_reads>0 && d.right.diagnostic.control_reads>0);

  /* Unsolicited, corrupt and timed-out readbacks cannot cause recovery. */
  setup_stopped(&d,&b);
  n=make_mode_reply(f,OID_ESC_MODE_CURRENT);
  FrontDrive_HandleRxFrame(&d,f,n,tick);
  assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  drop_reads=1;
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  f[n-1]^=1;
  FrontDrive_HandleRxFrame(&d,f,n,tick+1);
  assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  f[n-1]^=1;
  FrontDrive_HandleRxFrame(&d,f,n,tick+50);
  assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  assert(d.left.diagnostic.control_reads==0);

  /* A requested response after unlocking only updates diagnostics: never
     change modes online. The next locked read can trigger safe recovery. */
  setup_stopped(&d,&b); drop_reads=1;
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  d.control_read_allowed=0;
  FrontDrive_HandleRxFrame(&d,f,n,tick+1);
  assert(d.left.diagnostic.control_reads==1);
  assert(d.safety_state==FRONT_DRIVE_SAFETY_IDLE);
  assert(d.left_control_mode_cache==OID_ESC_MODE_SPEED);
  reads=read_count[0]+read_count[1];
  for(tick=101;tick<2101;tick++)
  {
    if(tick%10==0) (void)FrontDrive_SetTargetErpm(&d,400,400);
    FrontDrive_Task(&d,tick);
  }
  assert(read_count[0]+read_count[1]==reads);
  assert(target[0]==400 && target[1]==400);
  assert(max_hb_gap[0]<150 && max_hb_gap[1]<150);
  puts("PASS: fresh locked mode mismatch clears both targets, maintains heartbeat, waits for explicit rearm, and rejects stale/unlocked readback recovery; active driving starts no extra reads.");
  return 0;
}
