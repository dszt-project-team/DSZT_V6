/* Real OID protocol + scheduler, mocked UART wire and driver registers. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../modules/protocol/modbus_rtu.c"
#include "../modules/drive/oid_esc.c"
#include "../modules/drive/front_drive.c"
#include "../application/chassis/oid_stop_guard.h"

static uint32_t tick;
static int32_t target[2];
static uint16_t mode[2];
static uint8_t drop_write_id, drop_all, drop_reads, drop_status;
static uint32_t read_count[2], hb_count[2];
static uint32_t last_hb[2], max_hb_gap[2], status_count[2];
static uint16_t hb_value[2];
static uint8_t delayed_frame[32];
static uint16_t delayed_len;
static uint32_t delayed_due, response_delay;
uint32_t HAL_GetTick(void) { return tick; }
void BSP_RS485_PollFrameTimeout(BSP_RS485_Bus_t *b, uint32_t n, uint32_t gap)
{
  (void)gap;
  if(delayed_len && (int32_t)(n-delayed_due)>=0)
  {
    assert(b->rx_len==0);
    memcpy((void *)b->rx_buffer,delayed_frame,delayed_len);
    b->rx_len=delayed_len; b->frame_ready=1; delayed_len=0;
  }
}
uint16_t BSP_RS485_GetFrame(BSP_RS485_Bus_t *b, uint8_t *out, uint16_t size)
{
  uint16_t n = b->rx_len;
  assert(n <= size);
  memcpy(out, (const void *)b->rx_buffer, n);
  b->rx_len = 0; b->frame_ready = 0;
  return n;
}
static void put32(uint8_t *p, int32_t v)
{ uint32_t u=(uint32_t)v; p[0]=u>>24; p[1]=u>>16; p[2]=u>>8; p[3]=u; }
static uint16_t crc(uint8_t *p, uint16_t n)
{ uint16_t c=Modbus_RtuCrc16(p,n); p[n]=c; p[n+1]=c>>8; return n+2; }
HAL_StatusTypeDef BSP_RS485_Send(BSP_RS485_Bus_t *b, const uint8_t *p, uint16_t n, uint32_t timeout)
{
  uint8_t reply[32]={0};
  uint16_t len=0, reg=Modbus_RtuReadU16Be(p+2);
  uint8_t side=(uint8_t)(p[0]-1U);
  (void)timeout;
  assert(side<2 && Modbus_RtuCheckFrame(p,n));
  assert(b->rx_len==0); /* No overlapping response/request. */
  assert(delayed_len==0); /* Empty RX is not permission to preempt a slow reply. */
  if (drop_all) return HAL_OK;
  reply[0]=p[0]; reply[1]=p[1];
  if (p[1]==0x10)
  {
    assert(reg==OID_ESC_REG_TARGET_SPEED && n==13 && p[5]==2);
    if (p[0]==drop_write_id) return HAL_OK; /* UART success, wire lost. */
    target[side]=Modbus_RtuReadI32Be(p+7);
    memcpy(reply,p,6); len=crc(reply,6);
  }
  else if (p[1]==0x06)
  {
    if (reg==OID_ESC_REG_CONTROL_MODE) mode[side]=Modbus_RtuReadU16Be(p+4);
    if (reg==OID_ESC_REG_HEARTBEAT)
    {
      uint16_t v=Modbus_RtuReadU16Be(p+4);
      uint32_t gap=tick-last_hb[side];
      assert(v==1 || v==2);
      assert(v!=hb_value[side]); hb_value[side]=v;
      if(last_hb[side] && gap>max_hb_gap[side]) max_hb_gap[side]=gap;
      last_hb[side]=tick; hb_count[side]++;
    }
    memcpy(reply,p,n); len=n;
  }
  else if (p[1]==0x03)
  {
    assert(reg==0x1771 && p[5]==4);
    read_count[side]++;
    if (drop_reads) return HAL_OK;
    reply[2]=8; reply[3]=mode[side]>>8; reply[4]=mode[side];
    put32(reply+7,target[side]); len=crc(reply,11);
  }
  else
  {
    assert(p[1]==0x04 && reg==0x1388 && p[5]==9);
    status_count[side]++;
    if(drop_status) return HAL_OK;
    /* A target register can be nonzero while the controller is not in speed
       mode. Model actual speed separately for the cache-mismatch probe. */
    reply[2]=18;
    put32(reply+5,mode[side]==OID_ESC_MODE_SPEED ? target[side] : 0);
    len=crc(reply,21);
  }
  if(response_delay && (p[1]==0x03 || p[1]==0x04))
  {
    memcpy(delayed_frame,reply,len); delayed_len=len;
    delayed_due=tick+response_delay;
  }
  else
  { memcpy((void *)b->rx_buffer,reply,len); b->rx_len=len; b->frame_ready=1; }
  return HAL_OK;
}
static void consume(FrontDrive_Handle_t *d)
{
  uint8_t f[128]; uint16_t n=BSP_RS485_GetFrame(d->bus,f,sizeof(f));
  if(n) FrontDrive_HandleRxFrame(d,f,n,tick);
}
static void init_ready(FrontDrive_Handle_t *d, BSP_RS485_Bus_t *b)
{
  memset(b,0,sizeof(*b)); memset(d,0,sizeof(*d));
  FrontDrive_Init(d,b,1,2,14); tick=100;
  d->left_last_heartbeat_ms=d->right_last_heartbeat_ms=tick;
  d->left.status.last_update_ms=d->right.status.last_update_ms=tick;
  d->left_control_mode_cache=d->right_control_mode_cache=OID_ESC_MODE_SPEED;
  d->z_phase_start_ms=tick;
  target[0]=target[1]=400; mode[0]=mode[1]=1;
  drop_write_id=drop_all=drop_reads=drop_status=0;
  memset(hb_value,0,sizeof(hb_value)); memset(last_hb,0,sizeof(last_hb));
  memset(max_hb_gap,0,sizeof(max_hb_gap)); memset(status_count,0,sizeof(status_count));
  memset(hb_count,0,sizeof(hb_count)); delayed_len=0; response_delay=0;
}
int main(void)
{
  FrontDrive_Handle_t d; BSP_RS485_Bus_t b; uint8_t f[32]={1,3,8};
  uint16_t n; uint32_t i;
  init_ready(&d,&b);
  /* Stop sequence normally clears both; first-side choice does not matter. */
  for(i=0;i<2;i++)
  {
    d.speed_first_side=(uint8_t)i; target[0]=target[1]=400;
    assert(FrontDrive_StopUrgent(&d)==HAL_OK);
    tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
    tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
    assert(!d.command_pending && !target[0] && !target[1]);
  }
  /* Reproduce the known blind spot, not a claim of the physical root cause. */
  target[0]=target[1]=400; drop_write_id=1;
  assert(FrontDrive_StopUrgent(&d)==HAL_OK);
  tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
  tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
  assert(!d.command_pending && target[0]==400 && target[1]==0);
  tick+=51; OID_ESC_PollDiagnostic(&d.left,tick);
  assert(d.left.diagnostic.speed_unconfirmed==1);
  assert(d.left.diagnostic.zero_writes==3 && d.left.diagnostic.speed_acks==2);
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK); consume(&d);
  assert(d.left.diagnostic.target_erpm==400 && d.left.diagnostic.control_mode==1);
  assert(d.left.status.last_update_ms==100); /* Readback cannot fake online. */
  drop_write_id=0;

  /* Unsolicited, wrong-ID, corrupt, short and late replies cannot refresh data. */
  f[3]=0; f[4]=2; put32(f+7,-4900); n=crc(f,11);
  OID_ESC_HandleFrame(&d.left,f,n,++tick);
  assert(d.left.diagnostic.target_erpm==400);
  drop_reads=1;
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_BUSY);
  f[0]=2; n=crc(f,11); assert(!OID_ESC_HandleFrame(&d.left,f,n,tick));
  f[0]=1; n=crc(f,11); f[n-1]^=1;
  OID_ESC_HandleFrame(&d.left,f,n,tick); assert(d.left.diagnostic.control_pending);
  n=crc(f,9); OID_ESC_HandleFrame(&d.left,f,n,tick); assert(d.left.diagnostic.control_pending);
  put32(f+7,-4900); n=crc(f,11); OID_ESC_HandleFrame(&d.left,f,n,tick+50);
  assert(d.left.diagnostic.control_timeouts==1 && d.left.diagnostic.target_erpm==400);
  tick+=60; assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  OID_ESC_HandleFrame(&d.left,f,n,tick+1);
  assert(d.left.diagnostic.target_erpm==-4900 && d.left.diagnostic.control_mode==2);
  tick+=10; assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  f[1]=0x83; f[2]=2; n=crc(f,3);
  OID_ESC_HandleFrame(&d.left,f,n,tick+1);
  assert(!d.left.diagnostic.control_pending && d.left.diagnostic.exceptions==1);
  /* Unsigned deadline subtraction also works across tick rollover. */
  tick=UINT32_MAX-20U; assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  OID_ESC_PollDiagnostic(&d.left,29); assert(d.left.diagnostic.control_timeouts==2);

  /* Whole real scheduler: readback off by default, then alternating 500 ms/side. */
  init_ready(&d,&b); target[0]=target[1]=0;
  memset(read_count,0,sizeof(read_count)); memset(hb_count,0,sizeof(hb_count));
  for(tick=101;tick<1101;tick++) FrontDrive_Task(&d,tick);
  assert(read_count[0]==0 && read_count[1]==0);
  d.control_read_enabled=1;
  d.control_read_allowed=1;
  for(;tick<3101;tick++) FrontDrive_Task(&d,tick);
  assert(read_count[0]>=3 && read_count[1]>=3 && hb_count[0]>=20 && hb_count[1]>=20);
  assert(d.left.status.timeout_count==0 && d.right.status.timeout_count==0);
  assert(d.left.diagnostic.control_timeouts==0 && d.right.diagnostic.control_timeouts==0);
  drop_reads=1;
  for(;tick<5101;tick++) FrontDrive_Task(&d,tick);
  assert(d.left.diagnostic.control_timeouts>=3 && d.right.diagnostic.control_timeouts>=3);
  assert(FrontDrive_IsOnline(&d,FRONT_DRIVE_SIDE_LEFT,tick));
  assert(FrontDrive_IsOnline(&d,FRONT_DRIVE_SIDE_RIGHT,tick));
  assert(max_hb_gap[0]<200 && max_hb_gap[1]<200);

  /* Active driving: no extra readbacks, newest unsent target replaces old;
     partially sent pairs keep one coherent generation. */
  init_ready(&d,&b); d.control_read_enabled=1; d.control_read_allowed=0;
  memset(read_count,0,sizeof(read_count));
  assert(FrontDrive_SetTargetErpm(&d,100,100)==HAL_OK);
  assert(FrontDrive_SetTargetErpm(&d,200,200)==HAL_OK);
  tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
  assert(FrontDrive_SetTargetErpm(&d,300,300)==HAL_BUSY);
  tick+=10; FrontDrive_ProcessPendingCommand(&d,tick); consume(&d);
  assert(target[0]==200 && target[1]==200);
  for(;tick<4100;tick++)
  {
    if(tick%10==0) (void)FrontDrive_SetTargetErpm(&d,(int32_t)tick,(int32_t)tick);
    FrontDrive_Task(&d,tick);
  }
  assert(read_count[0]==0 && read_count[1]==0);
  assert(status_count[0]>25 && status_count[1]>25);
  assert(max_hb_gap[0]<150 && max_hb_gap[1]<150);
  assert(target[0]>3900 && target[1]>3900);
  assert(FrontDrive_StopUrgent(&d)==HAL_OK);
  for(;tick<4200;tick++) FrontDrive_Task(&d,tick);
  assert(target[0]==0 && target[1]==0);

  /* Repeated urgent zero and WAIT_REARM/WAIT_VERIFY cannot starve heartbeat. */
  for(i=FRONT_DRIVE_SAFETY_WAIT_REARM;i<=FRONT_DRIVE_SAFETY_WAIT_VERIFY;i++)
  {
    init_ready(&d,&b); d.safety_state=(uint8_t)i;
    for(tick=101;tick<2101;tick++)
    {
      assert(FrontDrive_SetTargetErpm(&d,400,400)==HAL_BUSY);
      FrontDrive_Task(&d,tick);
    }
    assert(target[0]==0 && target[1]==0);
    assert(hb_count[0]>10 && hb_count[1]>10);
    assert(max_hb_gap[0]<150 && max_hb_gap[1]<150);
    assert(d.safety_state==i);
  }
  /* Lost status responses still allow full-time heartbeat and repeated zeros. */
  init_ready(&d,&b); drop_status=1;
  for(tick=101;tick<4101;tick++) FrontDrive_Task(&d,tick);
  assert(d.safety_state==FRONT_DRIVE_SAFETY_WAIT_REARM);
  assert(target[0]==0 && target[1]==0);
  assert(max_hb_gap[0]<250 && max_hb_gap[1]<250);

  /* Already-issued 0x03 gets its bounded response window, never collision.
     A stop is delivered after timeout; no further diagnostic can start. */
  init_ready(&d,&b); drop_reads=1;
  assert(OID_ESC_RequestControl(&b,&d.left,tick)==HAL_OK);
  assert(FrontDrive_StopUrgent(&d)==HAL_OK);
  for(tick=101;tick<150;tick++) FrontDrive_Task(&d,tick);
  assert(target[0]==400 && target[1]==400);
  for(;tick<190;tick++) FrontDrive_Task(&d,tick);
  assert(target[0]==0 && target[1]==0);

  /* Slow valid responses, including stop during an empty-RX waiting period. */
  init_ready(&d,&b); response_delay=30;
  for(tick=101;tick<3000;tick++)
  {
    if(tick%10==0) (void)FrontDrive_SetTargetErpm(&d,500,500);
    FrontDrive_Task(&d,tick);
  }
  while(!delayed_len) FrontDrive_Task(&d,tick++);
  i=tick;
  assert(FrontDrive_StopUrgent(&d)==HAL_OK);
  for(;tick<i+100;tick++) FrontDrive_Task(&d,tick);
  assert(target[0]==0 && target[1]==0);
  assert(max_hb_gap[0]<200 && max_hb_gap[1]<200);
  assert(d.left.status.timeout_count==0 && d.right.status.timeout_count==0);

  /* Application guard + real stop sequencer: a lost zero is retried, not
     permanently suppressed by a one-shot application latch. */
  {
    OidStopGuard guard={0};
    init_ready(&d,&b); drop_write_id=1;
    for(tick=101;tick<500;tick++)
    {
      if(tick==200) drop_write_id=0;
      FrontDrive_Task(&d,tick);
      OidStopGuard_Update(&guard,1);
      if(OidStopGuard_RequestDue(&guard,tick) && FrontDrive_StopUrgent(&d)==HAL_OK)
        OidStopGuard_MarkRequest(&guard,tick);
    }
    assert(target[0]==0 && target[1]==0);
    assert(d.left.diagnostic.zero_writes>=3 && d.right.diagnostic.zero_writes>=3);
  }
  puts("PASS: OID frames, zero retry, latest coherent pair, active readback gate, reversal prerequisites, delayed replies, status loss, heartbeat in WAIT_REARM/VERIFY and busy traffic");
  return 0;
}
