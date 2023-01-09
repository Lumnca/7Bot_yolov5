/****************************************************
* Copyright (C) 2016-2021 Pinecone.AI(松果机器人) - All Rights Reserved
* 
* Project: Tensorflow Garbage Sorting (Robot Hardware Ctrl)
* Version: 1.1 (Conveyor Sorting with CRC Protocol - JS Class)
* 
* Date: Mar. 9th, 2021
* Author: Jerry Peng
*  
* This file implements Serial Events & Hardware Ctrl Strategy
/***************************************************/


const BAUD_RATE = 115200; // baud rate of robot serial port
/*Register ID */
//ROM
const DEVICE_TYPE_ID = 0;
const VERSION_ID = 1;
const MAC_ID = 2;
//EEPROM
const REG_EEPROM_SIZE = 6;
const EEPROM_ID = 11;
const DEVICE_ID = 11;
const BAUDRATE_ID = 12;
const LEVER_OFFSET_ID = 13;
//RAM 
const EEPROM_LOCK_ID = 28;
const OFFSET_CLB_ID = 29;
const SORTING_ID = 30;
const BTN_ENABLE_ID = 66;
//
const FB_FREQ_ID = 67;
const EXE_STATUS_ID = 68;
const BIN_STATUS_ID = 69;

//serial communication
let dataBuf = [];
let beginFlag = false;
let instruction = 0;
let cnt = 0;
let rxBuf_pre;
let BEGIN_FLAG_0 = 0xAA;
let BEGIN_FLAG_1 = 0x77;
let comReg = [];

let waitForRespond = false;

class SortingBot {


    constructor(serial) {
        this.serial = serial;
        console.log("初始化成功!!!")
    }

    /* Methods */

    //////////////////////////////////////////////////////////////////////////////////////
    /*Register Functions */
    //////////////////////////////////////////////////////////////////////////////////////
    readReg(addr, len) {
        let command = [5];
        command[0] = 0xAA;
        command[1] = 0x77;
        command[2] = 0x03;
        command[3] = addr;
        command[4] = len;
        this.sendCommand(command);
    }

    writeReg2(addr, len, data) {
        let command = [];
        command[0] = 0xAA;
        command[1] = 0x77;
        command[2] = 0x04;
        command[3] = addr;
        command[4] = len;
        for (let i = 0; i < len; i++)
            command[i + 5] = data[i];
        console.log(command)
        this.sendCommand(command);
    }

    writeReg(addr, data) {
        let command = [5];
        command[0] = 0xAA;
        command[1] = 0x77;
        command[2] = 0x04;
        command[3] = addr;
        command[4] = 1;
        command[5] = data;
        this.sendCommand(command);
    }

    ///////////////////////////////////////////////////////////////////
    /* Get Functions */
    //////////////////////////////////////////////////////////////////

    //return: device code,
    async getDeviceCode() {
        this.readReg(DEVICE_TYPE_ID, 1);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        return comReg[DEVICE_TYPE_ID];
    }

    //return: version code, range:[0, 255] 
    async getVersion() {
        this.readReg(VERSION_ID, 1);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        let version = 0.0;
        version = comReg[VERSION_ID] / 10.0
        return version.toFixed(1);
    }

    //return: MAC address(length: 6 bytes)
    async getMAC() {
        this.readReg(MAC_ID, 6);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        let MAC = "";
        for (let i = 0; i < 6; i++)
            MAC += hex(comReg[MAC_ID + i], 2);
        return MAC.split("").reverse().join("");
    }

    // return: device ID, range:[0, 255] 
    async getID() {
        this.readReg(DEVICE_ID, 1);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        return comReg[DEVICE_ID];
    }

    // return: levers' offset angle, range:[0, 223]
    async getOffset() {
        this.readReg(LEVER_OFFSET_ID, 4);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        let offsets = [];
        for (let i = 0; i < 4; i++) {
            offsets[i] = comReg[LEVER_OFFSET_ID + i];
        }
        return offsets;
    }

    // return: 0-motion done, 1-in process
    async getExeStatus() {
        this.readReg(EXE_STATUS_ID, 1);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        return comReg[EXE_STATUS_ID];
    }

    // return: 垃圾桶状态，bit[1]~bit[4]分别对应1~4号垃圾桶； 0-未满载，1-满载【兼容垃圾桶V2.0版本】
    async getBinStatus() {
        this.readReg(BIN_STATUS_ID, 1);
        if (await this.waitResponse() == 0) console.log("Cannot get response from robot!!!");
        return comReg[BIN_STATUS_ID];
    }



    ///////////////////////////////////////////////////////////////////
    /* Set Functions */
    ///////////////////////////////////////////////////////////////////

    //set EEPROM write lock: 0-lock off(enable writting), 1-lock on(protect EEPROM)
    setLock(data) {
        this.writeReg(EEPROM_LOCK_ID, data);
    }

    // clear offsets
    clearOffsets() {
        this.writeReg(OFFSET_CLB_ID, 2);
    }

    // set lever to forceless status
    leverRelease() {
        this.writeReg(OFFSET_CLB_ID, 3);
    }

    //[function] Set Levers Offsets Command
    //[input]    1:  set offsets once.
    setOffsets() {
        this.setLock(0);
        delay(30);
        this.writeReg(OFFSET_CLB_ID, 1);
        delay(30);
        this.setLock(1);
    }

    //[function] Set sorting execution command
    //[input]  0: stop,  1~4:  sorting ID.
    setSorting(ID) {
        this.writeReg(SORTING_ID, ID);
    }

    //[function] set button status
    //[input]    0-disable 1-enable
    setBtn(btn) {
        this.writeReg(BTN_ENABLE_ID, btn);
    }

    // [function] set feedback freqency, Unit:Hz
    //[input]    0-none feedbacks
    setFB(fb) {
        this.writeReg(FB_FREQ_ID, fb);
    }

    //EEPROM data init, this function will erase offset data.
    EEPROMinit() {
        // int data[] = {0, 0};
        // setLock(0);
        // writeReg(EEPROM_ID, EEPROM_LEN, data);
        // setLock(1);
    }




    ///////////////////////////////////////////////////////////////////
    /* Robot Data Receive */
    ///////////////////////////////////////////////////////////////////

    //wait robot responds to master command 
    //return: 0-no respond(error), 1-respond(sucess)
    async waitResponse() {
        let respond = 0;
        for (let i = 0; i < 10; i++) {
            await delay(5);
            if (this.readSerialPort() == 0x03) {
                respond = 1;
                break;
            }
        }
        return respond;
    }

    //return: command type, 0-null
    // !!!!!!!!!! this.XXX does not work properly in following method
    readSerialPort() {
        // console.log("run readSerialPort");
        let returnValue = 0;
        while (serial.available() > 0) {

            // read data
            let rxBuf = serial.read();

            if (!beginFlag) {
                beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag
            } else {
                if (instruction == 0) instruction = rxBuf;
                else {
                    switch (instruction) {

                        // Read register
                        case 0x03:
                            dataBuf[cnt++] = rxBuf;
                            // get data length
                            if (cnt >= 2) {
                                if (cnt >= dataBuf[1] + 4) {
                                    beginFlag = false;
                                    instruction = 0;
                                    cnt = 0;
                                    let addr = dataBuf[0];
                                    let num = dataBuf[1];
                                    // data read
                                    let data_read = [5 + num];
                                    data_read[0] = 0xAA;
                                    data_read[1] = 0x77;
                                    data_read[2] = 0x03;
                                    data_read[3] = addr;
                                    data_read[4] = num;

                                    for (let i = 0; i < num; i++) {
                                        data_read[5 + i] = dataBuf[2 + i];
                                    }

                                    console.log(data_read);

                                    // CRC check
                                    let crc = CRC16Modbus(data_read);
                                    let high = crc >> 8;
                                    let low = crc % 256;

                                    // CRC check pass, write reg
                                    if (low == dataBuf[num + 2] && high == dataBuf[num + 3]) {
                                        // update comReg
                                        for (let i = addr; i < addr + num; i++) {
                                            comReg[i] = dataBuf[2 + i - addr];
                                        }
                                        // recieve new register message
                                        // console.log("FB data recieved");
                                        //return 0x03;
                                        waitForRespond = false;
                                        returnValue = 0x03;
                                    }
                                }
                            }
                            break;

                            // auto feedbacks from robot
                        case 0x05:
                            dataBuf[cnt++] = rxBuf;
                            // get data length
                            if (cnt >= 2) {
                                if (cnt >= dataBuf[1] + 4) {
                                    beginFlag = false;
                                    instruction = 0;
                                    cnt = 0;
                                    let addr = dataBuf[0];
                                    let num = dataBuf[1];
                                    // data read
                                    let data_read = [5 + num];
                                    data_read[0] = 0xAA;
                                    data_read[1] = 0x77;
                                    data_read[2] = 0x05;
                                    data_read[3] = addr;
                                    data_read[4] = num;

                                    for (let i = 0; i < num; i++) {
                                        data_read[5 + i] = dataBuf[2 + i];
                                    }
                                    console.log(data_read);
                                    // CRC check
                                    let crc = CRC16Modbus(data_read);
                                    let high = crc >> 8;
                                    let low = crc % 256;

                                    // CRC check pass, write reg
                                    if (low == dataBuf[num + 2] && high == dataBuf[num + 3]) {
                                        // update comReg
                                        for (let i = addr; i < addr + num; i++) {
                                            comReg[i] = dataBuf[2 + i - addr];

                                            // XXX
                                            // if (i == XXX_ID) XXX = comReg[XXX_ID];
                                        }
                                        // recieve new auto feedback message
                                        //return 0x05;
                                        returnValue = 0x05;
                                    }
                                }
                            }
                            break;

                            // alarm
                        case 0x08:
                            dataBuf[cnt++] = rxBuf;
                            if (cnt >= 4) {
                                beginFlag = false;
                                instruction = 0;
                                cnt = 0;
                                let data_read = [0xAA, 0x77, 0x08, dataBuf[0], dataBuf[1]];
                                //CRC check
                                let crc = CRC16Modbus(data_read);
                                let high = crc / 256;
                                let low = crc % 256;
                                let level = dataBuf[0];
                                let type = dataBuf[1];

                                //CRC check pass, write reg
                                if (low == dataBuf[2] && high == dataBuf[3]) {
                                    console.log("ALARM!!! Level:", level, "  Type:", type);
                                    // recieve new alarm message
                                    //return 0x08;
                                    returnValue = 0x08;
                                }
                            }
                            break;

                        default:
                            beginFlag = false;
                            instruction = 0;
                            cnt = 0;
                            //return 0;
                            returnValue = 0x00;
                    }
                }
            }

            rxBuf_pre = rxBuf;
        } //while end

        return returnValue;
    }


    //////////////////////////////////////////////////////////////////////////////////////
    /*CRC */
    //////////////////////////////////////////////////////////////////////////////////////
    sendCommand(Msg) {
        let crc = this.CRC16Modbus(Msg);
        let high = crc >> 8;
        let low = crc % 256;
        for (let i = 0; i < Msg.length; i++) {
            this.serial.write(Msg[i]);
        }
        this.serial.write(low);
        this.serial.write(high);
        // console.log("CRC send: ", low, high);
        // console.log(this.serial);
    }

    CRC16Modbus(dataMsg) {
        let dataLen = dataMsg.length;
        let wCRCin = 0xFFFF;
        let wCPoly = 0x8005;
        let wbyte = 0;
        let j = 0;
        while (dataLen > 0) {
            dataLen--;
            wbyte = dataMsg[j++];
            wbyte = this.InvertUint8(wbyte);
            wCRCin ^= (wbyte << 8);
            for (let i = 0; i < 8; i++) {
                if ((wCRCin & 0x8000) != 0)
                    wCRCin = (wCRCin << 1) ^ wCPoly;
                else
                    wCRCin = wCRCin << 1;
            }
        }
        wCRCin = this.InvertUint16(wCRCin);
        return (wCRCin);
    }

    InvertUint16(dBuf) {
        let tmp = 0;
        for (let i = 0; i < 16; i++) {
            if ((dBuf & (1 << i)) != 0)
                tmp |= 1 << (15 - i);
        }
        return tmp;
    }

    InvertUint8(dBuf) {
        let tmp = 0;
        for (let i = 0; i < 8; i++) {
            if ((dBuf & (1 << i)) != 0)
                tmp |= 1 << (7 - i);
        }
        return tmp;
    }
} // class end

function CRC16Modbus(dataMsg) {
    let dataLen = dataMsg.length;
    let wCRCin = 0xFFFF;
    let wCPoly = 0x8005;
    let wbyte = 0;
    let j = 0;
    while (dataLen > 0) {
        dataLen--;
        wbyte = dataMsg[j++];
        wbyte = InvertUint8(wbyte);
        wCRCin ^= (wbyte << 8);
        for (let i = 0; i < 8; i++) {
            if ((wCRCin & 0x8000) != 0)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }
    wCRCin = InvertUint16(wCRCin);
    return (wCRCin);
}

function InvertUint16(dBuf) {
    let tmp = 0;
    for (let i = 0; i < 16; i++) {
        if ((dBuf & (1 << i)) != 0)
            tmp |= 1 << (15 - i);
    }
    return tmp;
}

function InvertUint8(dBuf) {
    let tmp = 0;
    for (let i = 0; i < 8; i++) {
        if ((dBuf & (1 << i)) != 0)
            tmp |= 1 << (7 - i);
    }
    return tmp;
}

function delay(ms) {
    return new Promise(resolve =>
        setTimeout(resolve, ms));
}

/* Serial Port */
let serial; // variable to hold an instance of the serialport library
let menu;
let inData; // for incoming serial data
let inByte;
let byteCount = 0;

let options = {
    baudRate: 115200
};
lists = []
/* Serial Port */
function printList(serialList) {
    
    for (let i = 0; i < serialList.length; i++) {
        var x = document.getElementById("device-port");
        var option = document.createElement("option");
        option.text = serialList[i];
        //x.add(option);
    }
    lists = serialList
}

function keyPressed() {
    serial.write(key);
    print("key:", key);
    if (key == '1') bot.setSorting(1);
    else if (key == '2') bot.setSorting(2);
    else if (key == '3') bot.setSorting(3);
    else if (key == '4') bot.setSorting(4);
    else bot.setSorting(0);
}

function serialEvent() {
    // read a byte from the serial port:
    // inByte = int(serial.read());
    // byteCount++;
    inByte = int(serial.read());
    if(inByte>-1){

        console.log(inByte)
    }
    
}

function serialError(err) {
    alart('Something went wrong with the serial port. ' + err);
}

// Connected to serial device
function gotOpen() {
    print("Serial Port is Open");
    serial.write('s');
    serial.write('0');
}


// detection filter
let class_id_list = [];

function setup() {
    // Processing JS to init its UI element

    // Serial Port
    serial = new p5.SerialPort(); // make a new instance of the serialport 
    serial.list();
    serial.on('data', serialEvent); // callback for when new data arrives
    serial.on('list', printList);
    serial.on('error', serialError); // callback for errors
    serial.on('open', gotOpen); // When our serial port is opened and ready for read/write
    serial.clear();
    //

    // filter init
    for (let i = 0; i < 20; i++) {
        class_id_list.push('0');
    }

}





/////////////////////////////////////////////////////////////////////////////////////////////
/*  hardware control   */



let motionFlag = false; // in motion process or not flag
let motionFlag_pre = false; // 
let detectTime = 0;
let motionTime = 0; // Unit:ms
let sortingID = 0;

let detectionFlag = 0;


function sorting(classId, probability) {

    // detection filter
    if (detectionFlag < 1) detectionFlag++; // sample every 3 detection
    else {
        if (!motionFlag) {
            detectionFlag = 0;
            class_id_list.splice(0, 1);
            class_id_list.push(classId);
        }

        if (!motionFlag && (probability > 85) && (list_compare(class_id_list, classId) > 18)) {
            console.log(classId);
            // 3- sorting ID
            if (classId == 1) { //'可回收物'
                motionFlag = true;
                detectTime = int(millis());
                sortingID = 1;
                motionTime = 8000;
            } else if (classId == 2) { //'有害垃圾'
                motionFlag = true;
                detectTime = int(millis());
                sortingID = 2;
                motionTime = 10000;
            } else if (classId == 3) { //'厨余垃圾'
                motionFlag = true;
                detectTime = int(millis());
                sortingID = 3;
                motionTime = 12000;
            } else if (classId == 4) { //'其它垃圾'
                motionFlag = true;
                detectTime = int(millis());
                sortingID = 4;
                motionTime = 14000;
            }
        }
    }


    // sorting begin
    if (motionFlag && !motionFlag_pre) {
        bot.setSorting(sortingID);
    }

    // sorting end
    if (motionFlag && int(millis()) - detectTime > motionTime) {
        motionFlag = false;
        bot.setSorting(0);
        sortingID = 0;

    }

    motionFlag_pre = motionFlag;
}


function list_compare(List, compareor) {
    let sum = 0;
    for (let i = 0; i < 20; i++) {
        if (List[i] == compareor)
            sum = sum + 1;
    }
    return sum;
}