/*
 *  Copyright (c) 2015, Ivor Wanders
 *  MIT License, see the LICENSE.md file in the root folder.
*/
#include "OBD9141.h"

OBD9141::OBD9141(){};

void OBD9141::begin(OBD_SERIAL_DATA_TYPE & serial_port, uint8_t rx_pin, uint8_t tx_pin){
    this->serial = &serial_port;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;

    // Enable the pullup on the Rx Pin, this is not changed by set_port.
    pinMode(this->rx_pin, INPUT);
    digitalWrite(this->rx_pin, HIGH);
    this->set_port(true); // prevents calling this->serial->end() before start.
    use_kwp_ = false;
}

void OBD9141::set_port(bool enabled){
    if (enabled){
        // Work around the incorrect pinmode configuration in Due.
        #ifdef ARDUINO_SAM_DUE
          g_APinDescription[this->rx_pin].pPort -> PIO_PDR = g_APinDescription[this->rx_pin].ulPin;
          g_APinDescription[this->tx_pin].pPort -> PIO_PDR = g_APinDescription[this->tx_pin].ulPin;
        #endif
        this->serial->begin(OBD9141_KLINE_BAUD);
    } else {
        this->serial->end();
        #ifdef ARDUINO_SAM_DUE
          g_APinDescription[this->rx_pin].pPort -> PIO_PER = g_APinDescription[this->rx_pin].ulPin; 
          g_APinDescription[this->tx_pin].pPort -> PIO_PER = g_APinDescription[this->tx_pin].ulPin; 
        #endif
        pinMode(this->tx_pin, OUTPUT);
        digitalWrite(this->tx_pin, HIGH);
    }
}

void OBD9141::kline(bool enabled){
    digitalWrite(this->tx_pin, enabled);
}


uint8_t OBD9141::checksum(void* b, uint8_t len){
    uint8_t ret = 0;
    for (uint8_t i=0; i < len; i++){
        ret += reinterpret_cast<uint8_t*>(b)[i];
    }
    return ret;
}

void OBD9141::write(uint8_t b){
    // OBD9141print("w: "); OBD9141println(b);
    this->serial->write(b);
    
    this->serial->setTimeout(OBD9141_REQUEST_ECHO_MS_PER_BYTE * 1 + OBD9141_WAIT_FOR_ECHO_TIMEOUT);
    uint8_t tmp[1]; // temporary variable to read into.
    this->serial->readBytes(tmp, 1);
}

void OBD9141::write(void* b, uint8_t len){
    for (uint8_t i=0; i < len ; i++){
        // OBD9141print("w: ");OBD9141println(reinterpret_cast<uint8_t*>(b)[i]);
        this->serial->write(reinterpret_cast<uint8_t*>(b)[i]);
        delay(OBD9141_INTERSYMBOL_WAIT);
    }
    this->serial->setTimeout(OBD9141_REQUEST_ECHO_MS_PER_BYTE * len + OBD9141_WAIT_FOR_ECHO_TIMEOUT);
    uint8_t tmp[len]; // temporary variable to read into.
    this->serial->readBytes(tmp, len);
}

// ------------------- write_stMach --------------------
// This is a state machine. Calls must follow a sequence.
// First call is with resetStMach asserted.
// Then writeStMach must be called succesively, with resetStMach de-asserted
// until the function returns true. ie processing is complete
// The parameters b and len only need to contain the correct contents on the first call.
// For this function to be non-blocking, I assume that the 64-byte serial buffer 
// will always have enough space to accept the serial chars.
enum writeStates {WRITE_IDLE, WRITE_START, WRITE_IS_DELAY, WRITE_NEXT_CHAR, WRITE_READ_ECHO };
int writeCurrSt = WRITE_IDLE;
unsigned long writeStartMillis = 0;
uint8_t writeIndex = 0;
uint8_t write_stMach_len = 0;
bool OBD9141::write_stMach(bool resetStMach, void* b, uint8_t len){
  bool writeDone = false;

  if (resetStMach)
    writeCurrSt = WRITE_START;
  switch (writeCurrSt) {
    case WRITE_IDLE:
      // do nothing. Wait here until start requested
      break;
    case WRITE_START:
      // copy b and len to local static copies. Dangerous stuff!
      // write one char to serial output
      writeIndex = 0;
      write_stMach_len = len;
      memcpy(write_stMach_buffer, b, write_stMach_len);
      this->serial->write(write_stMach_buffer[writeIndex++]);
      writeStartMillis = millis();
      writeCurrSt = WRITE_IS_DELAY;
      break;
    case WRITE_IS_DELAY:
      // Wait for OBD9141_INTERSYMBOL_WAIT mS before sending the next char.
      // When wait is over, test for write complete
      if (millis() - writeStartMillis >= OBD9141_INTERSYMBOL_WAIT) {
        if (writeIndex ==  write_stMach_len) {
          writeCurrSt = WRITE_READ_ECHO;
          writeStartMillis = millis();
	}
        else
          writeCurrSt = WRITE_NEXT_CHAR;
      }
      break;
    case WRITE_NEXT_CHAR:
      // Write one char to serial output
      this->serial->write(write_stMach_buffer[writeIndex++]);
      writeStartMillis = millis();
      writeCurrSt = WRITE_IS_DELAY;
      break;
    case WRITE_READ_ECHO:
      // Read back all the char echos, or timeout if all the chars are not received
      // Either way this function is done
      if (this->serial->available() >= write_stMach_len) {
        uint8_t tmp[write_stMach_len]; // temporary variable to read into.
        this->serial->readBytes(tmp, write_stMach_len);
        writeCurrSt = WRITE_IDLE;
        writeDone = true;
      }    
      else if (millis() - writeStartMillis >= (unsigned long) (OBD9141_REQUEST_ECHO_MS_PER_BYTE * write_stMach_len + 
				                                 OBD9141_WAIT_FOR_ECHO_TIMEOUT)) {
        OBD9141println("Timeout on echo read.");
        writeCurrSt = WRITE_IDLE;
        writeDone = true;
      }
      break;
  }
  return writeDone;
}



bool OBD9141::request(void* request, uint8_t request_len, uint8_t ret_len)
{
  if (use_kwp_)
  {
    // have to modify the first bytes.
    uint8_t rbuf[request_len];
    memcpy(rbuf, request, request_len);
    // now we modify the header, the payload is the request_len - 3 header bytes
    rbuf[0] = (0b11<<6) | (request_len - 3);
    rbuf[1] = 0x33;  // second byte should be 0x33
    return (this->requestKWP(&rbuf, request_len) == ret_len);
  }
  return request9141(request, request_len, ret_len);
}

bool OBD9141::request9141(void* request, uint8_t request_len, uint8_t ret_len){
    uint8_t buf[request_len+1];
    memcpy(buf, request, request_len); // copy request

    buf[request_len] = this->checksum(&buf, request_len); // add the checksum

    this->write(&buf, request_len+1);

    // wait after the request, officially 30 ms, but we might as well wait
    // for the data in the readBytes function.
    
    // set proper timeout
    this->serial->setTimeout(OBD9141_REQUEST_ANSWER_MS_PER_BYTE * ret_len + OBD9141_WAIT_FOR_REQUEST_ANSWER_TIMEOUT);
    memset(this->buffer, 0, ret_len+1);
    
    OBD9141print("Trying to get x bytes: "); OBD9141println(ret_len+1);
    if (this->serial->readBytes(this->buffer, ret_len+1)){
        // OBD9141print("R: ");
        // for (uint8_t i=0; i< (ret_len+1); i++){
            // OBD9141print(this->buffer[i]);OBD9141print(" ");
        // };OBD9141println();
        
        return (this->checksum(&(this->buffer[0]), ret_len) == this->buffer[ret_len]);// have data; return whether it is valid.
    } else {
        OBD9141println("Timeout on reading bytes.");
        return false; // failed getting data.
    }
}

// request9141_stMach is a state machine. Calls must follow a sequence.
// First call requires resetStMach to be asserted.
// Then repeated calls, with resetStMach de-asserted, until the function returns true. ie processing is complete.
// Then the caller must check the state of "success" to determine if the request was successful.
// Only the first  call must have pid, mode and return_length correctly set.
enum reqStates {REQ_IDLE, REQ_START, REQ_WAIT_DATA, REQ_WAIT_WRITE_DONE};
int reqCurrSt = REQ_IDLE;
unsigned long reqStartMillis = 0;
uint8_t req_ret_len;
uint8_t req_pid;
bool OBD9141::request9141_stMach(bool resetStMach, bool *success, uint8_t pid, uint8_t mode, uint8_t return_length){
  uint8_t request_len = 5;
  bool reqDone = false;
  *success = false;
  uint8_t buf[request_len+1];

  if (resetStMach)
    reqCurrSt = REQ_START;
  switch (reqCurrSt) {
    case REQ_IDLE:
      // do nothing. Wait here until start requested
    break;
    case REQ_START: {
      // Copy parameters that are used on multiple passes to static variables. Dangerous stuff!
      // Format the TX message, and and start the write_stMack.
      req_ret_len = return_length + 5;
      req_pid = pid;
      // header of request is 5 long, first three are always constant.
      uint8_t message[5] = {0x68, 0x6A, 0xF1, mode, req_pid};
      memcpy(buf, message, request_len); // copy request
      buf[request_len] = this->checksum(&buf, request_len); // add the checksum
      this->write_stMach(true, &buf, request_len+1);
      reqCurrSt = REQ_WAIT_WRITE_DONE;
      }
      break;
    case REQ_WAIT_WRITE_DONE:
      // Keep calling write_stMach until all the chars have been sent and the echos received
      // When the first arguement of write_stMach is false the other arguements are not used				      
      if (this->write_stMach(false, &buf, 0)) {
        memset(this->buffer, 0, req_ret_len+1);
        reqStartMillis = millis();
        reqCurrSt = REQ_WAIT_DATA;
        OBD9141print("request9141_stMach: Trying to read: "); OBD9141print(req_ret_len+1); OBD9141println(" bytes");
      }
      break;
    case REQ_WAIT_DATA:
      // Wait for the echo, and read the chars, or time out if chars not received
      // wait after the request, officially 30 ms, but we might as well wait
      // for the data in the readBytes function.
      if (this->serial->available() >= req_ret_len+1) {
        this->serial->readBytes(this->buffer, req_ret_len+1);
        bool checksumPass = (this->checksum(&(this->buffer[0]), req_ret_len) == this->buffer[req_ret_len]);// check if valid.
        if (this->buffer[4] != req_pid || checksumPass == false)
          *success = false;
        else
          *success = true;
        reqDone = true;
        reqCurrSt = REQ_IDLE;
      }    
      else if (millis() - reqStartMillis >= (unsigned long) (OBD9141_REQUEST_ANSWER_MS_PER_BYTE * req_ret_len +
			       	OBD9141_WAIT_FOR_REQUEST_ANSWER_TIMEOUT)) {
        OBD9141println("Timeout on pid req response.");
        reqCurrSt = REQ_IDLE;
        *success = false;
        reqDone = true;
      }
      break;
  }
  return reqDone;
}

uint8_t OBD9141::request(void* request, uint8_t request_len){
    if (use_kwp_)
    {
        // have to modify the first bytes.
        uint8_t rbuf[request_len];
        memcpy(rbuf, request, request_len);
        // now we modify the header, the payload is the request_len - 3 header bytes
        rbuf[0] = (0b11<<6) | (request_len - 3);
        rbuf[1] = 0x33;  // second byte should be 0x33
        return requestKWP(rbuf, request_len);
    }
    bool success = true;
    // wipe the entire buffer to ensure we are in a clean slate.
    memset(this->buffer, 0, OBD9141_BUFFER_SIZE);

    // create the request with checksum.
    uint8_t buf[request_len+1];
    memcpy(buf, request, request_len); // copy request
    buf[request_len] = this->checksum(&buf, request_len); // add the checksum

    // manually write the bytes onto the serial port
    // this does NOT read the echoes.
    OBD9141print("W: ");
#ifdef OBD9141_DEBUG
    for (uint8_t i=0; i < (request_len+1); i++){
        OBD9141print(buf[i]);OBD9141print(" ");
    };OBD9141println();
#endif
    for (uint8_t i=0; i < request_len+1 ; i++){
        this->serial->write(reinterpret_cast<uint8_t*>(buf)[i]);
        delay(OBD9141_INTERSYMBOL_WAIT);
    }

    // next step, is to read the echo from the serial port.
    this->serial->setTimeout(OBD9141_REQUEST_ECHO_MS_PER_BYTE * 1 + OBD9141_WAIT_FOR_ECHO_TIMEOUT);
    uint8_t tmp[request_len+1]; // temporary variable to read into.
    this->serial->readBytes(tmp, request_len+1);

    OBD9141print("E: ");
    for (uint8_t i=0; i < request_len+1; i++)
    {
#ifdef OBD9141_DEBUG
        OBD9141print(tmp[i]);OBD9141print(" ");
#endif
      // check if echo is what we wanted to send
      success &= (buf[i] == tmp[i]);
    }

    // so echo is dealt with now... next is listening to the reply, which is a variable number.
    // set the timeout for the first read to include the wait for answer timeout
    this->serial->setTimeout(OBD9141_REQUEST_ANSWER_MS_PER_BYTE * 1 + OBD9141_WAIT_FOR_REQUEST_ANSWER_TIMEOUT);

    uint8_t answer_length = 0;
    // while readBytes returns a byte, keep reading.
    while (this->serial->readBytes(&(this->buffer[answer_length]), 1) && (answer_length < OBD9141_BUFFER_SIZE))
    {
      answer_length++;
      this->serial->setTimeout(OBD9141_REQUEST_ANSWER_MS_PER_BYTE * 1);
    }

    OBD9141println();OBD9141print("A (");OBD9141print(answer_length);OBD9141print("): ");
#ifdef OBD9141_DEBUG
    for (uint8_t i=0; i < min(answer_length, OBD9141_BUFFER_SIZE); i++){
        OBD9141print(this->buffer[i]);OBD9141print(" ");
    };OBD9141println();
#endif

    // next, calculate the checksum
    bool checksum = (this->checksum(&(this->buffer[0]), answer_length-1) == this->buffer[answer_length - 1]);
    OBD9141print("C: ");OBD9141println(checksum);
    OBD9141print("S: ");OBD9141println(success);
    OBD9141print("R: ");OBD9141println(answer_length - 1);
    if (checksum && success)
    {
      return answer_length - 1;
    }
    return 0;
}

uint8_t OBD9141::requestKWP(void* request, uint8_t request_len){
    uint8_t buf[request_len+1];
    memcpy(buf, request, request_len); // copy request

    buf[request_len] = this->checksum(&buf, request_len); // add the checksum

    this->write(&buf, request_len+1);

    // wait after the request, officially 30 ms, but we might as well wait
    // for the data in the readBytes function.
    
    // set proper timeout
    this->serial->setTimeout(OBD9141_REQUEST_ANSWER_MS_PER_BYTE * 1 + OBD9141_WAIT_FOR_REQUEST_ANSWER_TIMEOUT);
    memset(this->buffer, 0, OBD9141_BUFFER_SIZE);

    // Example response: 131 241 17 193 239 143 196 0 
    // Try to read the fmt byte.
    if (!(this->serial->readBytes(this->buffer, 1)))
    {
      return 0; // failed reading the response byte.
    }

    const uint8_t msg_len = (this->buffer[0]) & 0b111111;
    // If length is zero, there is a length byte at the end of the header.
    // This is likely very rare, we do not handle this for now.

    // This means that we should now read the 2 addressing bytes, the payload
    // and the checksum byte.
    const uint8_t remainder = msg_len + 2 + 1;
    OBD9141print("Rem: ");OBD9141println(remainder);
    const uint8_t ret_len = remainder + 1;
    OBD9141print("ret_len: ");OBD9141println(ret_len);
    this->serial->setTimeout(OBD9141_REQUEST_ANSWER_MS_PER_BYTE * (remainder + 1));

    //OBD9141print("Trying to get x bytes: "); OBD9141println(ret_len+1);
    if (this->serial->readBytes(&(this->buffer[1]), remainder)){
        OBD9141print("R: ");
        for (uint8_t i=0; i< (ret_len+1); i++){
            OBD9141print(this->buffer[i]);OBD9141print(" ");
        };OBD9141println();
  
        const uint8_t calc_checksum = this->checksum(&(this->buffer[0]), ret_len - 1);
        OBD9141print("calc cs: ");OBD9141println(calc_checksum);
        OBD9141print("buf cs: ");OBD9141println(this->buffer[ret_len - 1]);
        
        if (calc_checksum == this->buffer[ret_len - 1])
        {
          return ret_len - 1; // have data; return whether it is valid.
        }
    } else {
        OBD9141println("Timeout on reading bytes.");
        return 0; // failed getting data.
    }
    return 0;
}
/*
    No header description to be found on the internet?

    for 9141-2:
      raw request: {0x68, 0x6A, 0xF1, 0x01, 0x0D}
          returns:  0x48  0x6B  0x11  0x41  0x0D  0x00  0x12 
          returns 1 byte
          total of 7 bytes.

      raw request: {0x68, 0x6A, 0xF1, 0x01, 0x00}
          returns   0x48  0x6B  0x11  0x41  0x00  0xBE  0x3E  0xB8  0x11  0xCA
          returns 3 bytes
          total of 10 bytes

      Mode seems to be 0x40 + mode, unable to confirm this though.

    for ISO 14230 KWP:
      First byte lower 6 bits are length, first two bits always 0b11?

      raw request: {0xc2, 0x33, 0xf1, 0x01, 0x0d, 0xf4}
      returns       0x83  0xf1  0x11  0x41  0xd  0x0  0xd3

      raw request: {0xc2, 0x33, 0xf1, 0x01, 0x0c, 0xf3}
      returns       0x84, 0xf1, 0x11, 0x41, 0x0c, 0x0c, 0x4c, 0x2b, 0xf3
*/


bool OBD9141::getPID(uint8_t pid, uint8_t mode, uint8_t return_length){
    uint8_t message[5] = {0x68, 0x6A, 0xF1, mode, pid};
    // header of request is 5 long, first three are always constant.

    bool res = this->request(&message, 5, return_length+5);
    // checksum is already checked, verify the PID.

    if (this->buffer[4] != pid){
        return false;
    }
    return res;
}

bool OBD9141::getCurrentPID(uint8_t pid, uint8_t return_length){
    return this->getPID(pid, 0x01, return_length);
}

bool OBD9141::clearTroubleCodes(){
    uint8_t message[4] = {0x68, 0x6A, 0xF1, 0x04};
    // 0x04 without PID value should clear the trouble codes or
    // malfunction indicator lamp.

    // No data is returned to this request, we expect the request itself
    // to be returned.
    bool res = this->request(&message, 4, 4);
    return res;
}

uint8_t OBD9141::readTroubleCodes()
{
  uint8_t message[4] = {0x68, 0x6A, 0xF1, 0x03};
  uint8_t response = this->request(&message, 4);
  if (response >= 4)
  {
    // OBD9141print("T: ");OBD9141println((response - 4) / 2);
    return (response - 4) / 2;  // every DTC is 2 bytes, header was 4 bytes.
  }
  return 0;
}

uint8_t OBD9141::readPendingTroubleCodes()
{
  uint8_t message[4] = {0x68, 0x6A, 0xF1, 0x07};
  uint8_t response = this->request(&message, 4);
  if (response >= 4)
  {
    // OBD9141print("T: ");OBD9141println((response - 4) / 2);
    return (response - 4) / 2;  // every DTC is 2 bytes, header was 4 bytes.
  }
  return 0;
}

uint8_t OBD9141::readUint8(){
    return this->buffer[5];
}

uint16_t OBD9141::readUint16(){
    return this->buffer[5]*256 + this->buffer[6]; // need to reverse endianness
}

uint8_t OBD9141::readUint8(uint8_t index){
    return this->buffer[5 + index];
}

uint32_t OBD9141::readUint32(){
  return (uint32_t(this->buffer[5]) << 24) | (uint32_t(this->buffer[6]) << 16) | (uint32_t(this->buffer[7]) << 8) | this->buffer[8]; // need to reverse endianness
}

uint8_t OBD9141::readBuffer(uint8_t index){
  return this->buffer[index];
}

uint16_t OBD9141::getTroubleCode(uint8_t index)
{
  return *reinterpret_cast<uint16_t*>(&(this->buffer[index*2 + 4]));
}

#define ONE_BIT_AT_5_BAUD 200
enum initStates {INIT_IDLE, INIT_START, IDLE_BEFORE, MAGIC5_BIT_START, MAGIC5_BIT_START_DELAY,
       MAGIC5_BIT1_2, MAGIC5_BIT1_2_DELAY,
       MAGIC5_BIT3_4, MAGIC5_BIT3_4_DELAY ,
       MAGIC5_BIT5_6, MAGIC5_BIT5_6_DELAY,
       MAGIC5_BIT7_8, MAGIC5_BIT7_8_DELAY ,
       MAGIC5_BIT_STOP, MAGIC5_BIT_STOP_DELAY ,
       WAIT_CONF_WR_DONE, SET_SER_PORT , WAIT_0X55, WAIT_V1 , WAIT_V2 , WAIT_W4, WAIT_0XCC, SUCCESS_DELAY };
int initCurrSt = INIT_IDLE;
unsigned long startMillis = 0;
uint8_t v1=0, v2=0; // sent by car:  (either 0x08 or 0x94)

// init is a state machine. It is init'd to an idle state
// To start the state machine, call with resetStMach = true
// With successive calls, init will eventually return
// true to indicate that it has finished. The caller should then check
// the value of success to determine if init was successful
bool OBD9141::init(bool resetStMach, bool *success){
    bool initDone = false;
    *success = false;
    uint8_t buffer[1];

    if (resetStMach)
        initCurrSt = INIT_START;
    switch (initCurrSt) {
      case INIT_IDLE:
        // do nothing. Wait here until start requested
	break;
      case INIT_START:
        // Disable serial port, and enable bit bang 
        use_kwp_ = false;
        this->set_port(false); // disable the port.
        this->kline(true);
	initCurrSt = IDLE_BEFORE;
	startMillis = millis();
        OBD9141println("Before magic 5 baud.");
	break;
      case IDLE_BEFORE:
	// Idle for 3 seconds before starting tx of init char
        if (millis() - startMillis >= OBD9141_INIT_IDLE_BUS_BEFORE) // no traffic on bus for 3 seconds.
	    initCurrSt = MAGIC5_BIT_START;
        break;

     case MAGIC5_BIT_START:
        // Start the ISO9141 5-baud 'slow' init.
        this->kline(false); // start
	startMillis = millis();
	initCurrSt = MAGIC5_BIT_START_DELAY;
	break;
      case MAGIC5_BIT_START_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD )
	    initCurrSt = MAGIC5_BIT1_2;
        break;

     case MAGIC5_BIT1_2:
       	// first two bits
        this->kline(true);
	startMillis = millis();
	initCurrSt = MAGIC5_BIT1_2_DELAY;
	break;
      case MAGIC5_BIT1_2_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD * 2 )
	    initCurrSt = MAGIC5_BIT3_4;
        break;

     case MAGIC5_BIT3_4:
	// second pair
        this->kline(false); 
	startMillis = millis();
	initCurrSt = MAGIC5_BIT3_4_DELAY;
	break;
      case MAGIC5_BIT3_4_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD * 2 )
	    initCurrSt = MAGIC5_BIT5_6;
        break;

     case MAGIC5_BIT5_6:
       	// third pair
        this->kline(true);
	startMillis = millis();
	initCurrSt = MAGIC5_BIT5_6_DELAY;
	break;
      case MAGIC5_BIT5_6_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD * 2 )
	    initCurrSt = MAGIC5_BIT7_8;
        break;

     case MAGIC5_BIT7_8:
       	// last pair
        this->kline(false);
	startMillis = millis();
	initCurrSt = MAGIC5_BIT7_8_DELAY;
	break;
      case MAGIC5_BIT7_8_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD * 2 )
	    initCurrSt = MAGIC5_BIT_STOP;
        break;

     case MAGIC5_BIT_STOP:
	// Stop bit.
        // this last 200 ms delay could also be put in the setTimeout below.
        // But the spec says we have a stop bit.
        this->kline(true); // stop bit
	startMillis = millis();
	initCurrSt = MAGIC5_BIT_STOP_DELAY;
	break;
      case MAGIC5_BIT_STOP_DELAY:
        if (millis() - startMillis >= ONE_BIT_AT_5_BAUD )
	    initCurrSt = SET_SER_PORT;
        break;

      case SET_SER_PORT:
        // done, from now on it the bus can be treated ad a 10400 baud serial port.
        //OBD9141println("Before setting port.");
        this->set_port(true);
        OBD9141println("After setting port.");
	startMillis = millis();
	initCurrSt = WAIT_0X55;
	break;

      case WAIT_0X55:
	// Wait for 0x55 char from ECU or timeout
	if (this->serial->available() > 0) {
          // read first value into buffer, should be 0x55
          this->serial->readBytes(buffer, 1);
          OBD9141print("First read is: "); OBD9141println(buffer[0]);
          if (buffer[0] != 0x55){
	    initCurrSt = INIT_IDLE;
            *success = false;
	    initDone = true;
	  }
          else { 
	    initCurrSt = WAIT_V1;
	    startMillis = millis();
	  }
        }    
	else if (millis() - startMillis >= 300+200) {
            OBD9141println("Timeout on read 0x55.");
	    initCurrSt = INIT_IDLE;
	    *success = false;
	    initDone = true;
	}
        break;

      case WAIT_V1:
	// Wait for V1 or timeout
        // we get here after we have passed receiving the first 0x55 from ecu.
	if (this->serial->available() > 0) {
          // read v1 into buffer, should be 0x08 or 0x94
          this->serial->readBytes(buffer, 1);
          v1 = buffer[0];
          OBD9141print("read v1: "); OBD9141println(v1);
	  initCurrSt = WAIT_V2;
	  startMillis = millis();
        }    
	else if (millis() - startMillis >= 20) {
            OBD9141println("Timeout on read v1.");
	    initCurrSt = INIT_IDLE;
	    *success = false;
	    initDone = true;
	}
        break;

      case WAIT_V2:
	// Wait for V2 or timeout
	if (this->serial->available() > 0) {
          // read v2 into buffer, should be 0x08 or 0x94
          this->serial->readBytes(buffer, 1);
          v2 = buffer[0];
          OBD9141print("read v2: "); OBD9141println(v1);
          // these two should be identical according to the spec.
          if (v1 != v2){
	    *success = false;
	    initDone = true;
          }
	  else {
	    initCurrSt = WAIT_W4;
	    startMillis = millis();
	  }
        }    
	else if (millis() - startMillis >= 20) {
            OBD9141println("Timeout on read v2.");
	    initCurrSt = INIT_IDLE;
	    *success = false;
	    initDone = true;
	}
        break;

      case WAIT_W4:
        // we obtained v1 and v2, now invert and send it back.
        // tester waits w4 between 25 and 50 ms:
	if (millis() - startMillis >= 30) {
            buffer[0] = ~v2;
            this->write_stMach(true, &buffer[0], 1);
	    initCurrSt = WAIT_CONF_WR_DONE;
	    startMillis = millis();
	}
        break;
      case WAIT_CONF_WR_DONE:
        // Keep calling write_stMach until ~V2  has been sent and the echo received
        // When the first arguement of write_stMach is false the other arguements are not used				      
        if (this->write_stMach(false, &buffer[0], 0)) {
          startMillis = millis();
          initCurrSt = WAIT_0XCC;
        }
        break;
 
      case WAIT_0XCC:
        // finally, attempt to read 0xCC from the ECU, indicating succesful init.
	if (this->serial->available() > 0) {
          // read final value into buffer, should be 0xcc
          this->serial->readBytes(buffer, 1);
          OBD9141print("Final read is: "); OBD9141println(buffer[0]);
          if (buffer[0] != 0xcc){
	    initCurrSt = INIT_IDLE;
            *success = false;
	    initDone = true;
	  }
          else { 
	    initCurrSt = SUCCESS_DELAY;
	    startMillis = millis();
	  }
        }    
	else if (millis() - startMillis >= 50) {
            OBD9141println("Timeout on read 0xcc.");
	    initCurrSt = INIT_IDLE;
	    *success = false;
	    initDone = true;
	}
        break;

      case SUCCESS_DELAY:
        // this delay is not in the spec, but prevents requests immediately
        // after the finishing of the init sequency.
	if (millis() - startMillis >= OBD9141_INIT_POST_INIT_DELAY) {
	    initCurrSt = INIT_IDLE;
	    *success = true;
	    initDone = true;
	}
        break;
    }
    return initDone;
}

bool OBD9141::initKWP(){
    use_kwp_ = true;
    // this function performs the KWP2000 fast init.
    this->set_port(false); // disable the port.
    this->kline(true); // set to up
    delay(OBD9141_INIT_IDLE_BUS_BEFORE); // no traffic on bus for 3 seconds.
    OBD9141println("Before 25 ms / 25 ms startup.");
    this->kline(false); delay(25); // start with 25 ms low
    this->kline(true); delay(25); // 25 ms high.

    // immediately follow this by a startCommunicationRequest
    OBD9141println("Enable port.");
    this->set_port(true);

    // startCommunicationRequest message:
    uint8_t message[4] = {0xC1, 0x33, 0xF1, 0x81};
    // checksum (0x66) is calculated by request method.

    // Send this request and read the response
    if (this->requestKWP(&message, 4) == 6) {
        // check positive response service ID, should be 0xC1.
        if (this->buffer[3] == 0xC1) {
            // Not necessary to do anything with this data?
            return true;
        } else {
            return false;
        }
    }
    return false;
}

void OBD9141::decodeDTC(uint16_t input_bytes, uint8_t* output_string){
  const uint8_t A = reinterpret_cast<uint8_t*>(&input_bytes)[0];
  const uint8_t B = reinterpret_cast<uint8_t*>(&input_bytes)[1];
  const static char type_lookup[4] = {'P', 'C', 'B', 'U'};
  const static char digit_lookup[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

  // A7-A6 is first dtc character, error type:
  output_string[0] = type_lookup[A >> 6];
  // A5-A4 is second dtc character
  output_string[1] = digit_lookup[(A >> 4) & 0b11];
  // A3-A0 is third dtc character.
  output_string[2] = digit_lookup[A & 0b1111];
  // B7-B4 is fourth dtc character
  output_string[3] = digit_lookup[B >> 4];
  // B3-B0 is fifth dtc character
  output_string[4] = digit_lookup[B & 0b1111];
}
