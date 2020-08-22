int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  UNUSED(to_push);
  return true;
}

// *** no output safety mode ***

static void nooutput_init(int16_t param) {
  UNUSED(param);
  //controls_allowed = false;
  controls_allowed = true;
  relay_malfunction_reset();
}

// elm327 tx hook
static int nooutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return false;
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);

  //All ISO 15765-4 messages must be 8 bytes long
  if (len != 8) {
    tx = 0;
  }

  //Check valid 29 bit send addresses for ISO 15765-4
  //Check valid 11 bit send addresses for ISO 15765-4
  if ((addr != 0x18DB33F1) && ((addr & 0x1FFF00FF) != 0x18DA00F1) &&
      ((addr & 0x1FFFFF00) != 0x700)) {
    tx = 0;
  }
  return tx;
}


static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(to_fwd);
  int bus_fwd = -1; // no forward default
  
  if ((hw_type == HW_TYPE_WHITE_PANDA) || (hw_type == HW_TYPE_GREY_PANDA)){
    //puts("GREY or WHITE\n");
    if (bus_num == 0){
      bus_fwd = 2;
      //puts("FWD 0->2\n");
    } 
    if (bus_num == 2){
      bus_fwd = 0;
      //puts("FWD 2->0\n");
    }
  }

  return bus_fwd;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

static void alloutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};
