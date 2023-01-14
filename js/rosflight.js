'use-strict'

var MAV = {
    mavlinkParser:              0,
    state:                      0,
    id:                         1,
    comp:                       0,
    message_length_expected:    0,
    message_length_received:    0,
    message_buffer:             {},
    dataView:                   0,
    callbacks:                  [],
    unsupported:                0,
    last_received_timestamp:   null,
    listeners:                  [],

    init_parser: ()=>{
        if(!MAV.mavlinkParser){
            MAV.mavlinkParser = new MAVLink10Processor();
        }
    },
    read_calback: (msg)=>{
        if(msg == null) return null;
        else if(MAV.mavlinkParser.buf.byteOffset > 0){  
            for(var i=0;i<msg.length;i++){
                rosflightIO.handle_message(msg[i]);
            }
        }
    },
    read: function (readInfo) {
        MAV.init_parser();
        var mdata = readInfo.data;
        var u8 = new Uint8Array(mdata);
        MAV.message_buffer = MAV.mavlinkParser.parseBuffer(u8);
        //this.dataView = new DataView(this.message_buffer, 0, this.message_length_expected);
        MAV.last_received_timestamp = Date.now();
        MAV.read_calback(MAV.message_buffer)
    },
    set_write: (bufferOut)=>{
        MAV.init_parser();
        MAV.send_buffer = bufferOut;
        MAV.mavlinkParser.file = {
            write : (buf)=>{
                //var data = new Uint8Array(buf); 
                serial.send(buf, (info)=>{
                    console.log(buf);
                    if (sendInfo.bytesSent == MAV.send_buffer.byteLength) {
                        if (callback_sent) callback_sent();
                    }  
                });
            }
        };
        return MAV.mavlinkParser.file.write;
    },
    send_message: function (code, data, callback_sent, callback_msp, callback_onerror) {

        if (code === undefined || typeof mavlink10 ==='undefined') {
            debugger;
        } 
        if (!callback_onerror) {
            var callbackOnError = false;
        } else {
            var callbackOnError = true;
        }
        var bufferOut;
        
        switch (code)
        {
            case mavlink10.MAVLINK_MSG_ID_OFFBOARD_CONTROL:
                rosflightIO.handle_msg_callback_id = mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_STATUS; 
                bufferOut = new mavlink10.messages.offboard_control(data.mode, 0, data.x, data.y, data.z, data.F);
            break;
            case mavlink10.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                bufferOut = new mavlink10.messages.param_request_list(this.id, this.comp);
            break;
            case mavlink10.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                bufferOut = new mavlink10.messages.param_request_read(this.id, this.comp, data.param_id, data.param_index);
            break;
            case mavlink10.MAVLINK_MSG_ID_PARAM_SET:
                rosflightIO.handle_msg_callback_id = mavlink10.MAVLINK_MSG_ID_PARAM_VALUE;
                bufferOut = new mavlink10.messages.param_set(this.id, this.comp, data.param_id, data.param_value, data.param_type);
            break;
            case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD:
                bufferOut = new mavlink10.messages.rosflight_cmd(data);
            break; 
            case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD:
                bufferOut = new mavlink10.messages.rosflight_aux_cmd(data.type_array, data.aux_cmd_array)
            break;
            case mavlink10.MAVLINK_MSG_ID_TIMESYNC:
                bufferOut = new mavlink10.messages.timesync();
            break;
            case mavlink10.MAVLINK_MSG_ID_EXTERNAL_ATTITUDE:
                bufferOut = new mavlink10.messages.external_attitude();
            break;
            case mavlink10.MAVLINK_MSG_ID_HEARTBEAT:
                bufferOut = new mavlink10.messages.heartbeat(this.id, this.comp);
            break;
            case mavlink10.MAVLINK_MSG_ID_PARAM_MAP_RC:
                //bufferOut = new mavlink10.messages.param_map_rc(this.id, this.comp, data.param_id, data.param_index, data.parameter_rc_channel_index, data.param_value0, data.scale, data.param_value_min, data.param_value_max);
                break;
            default:
                data = false;
                rosflightIO.handle_msg_callback_id = code;
            break;
        }

       //var obj = {'code': code, 'requestBuffer': buf, 'callback': (callback_msp) ? callback_msp : false, 'timer': false, 'callbackOnError': callbackOnError};
       var requestExists = false;
       /* for (var i = 0; i < MAV.callbacks.length; i++) {
            if (MAV.callbacks[i].code == code) {
                // request already exist, we will just attach
                requestExists = true;
                break;
            }
        }

        if (!requestExists) {
            obj.timer = setInterval(function () {
                console.log('MSP data request timed-out: ' + code);
                MAV.mavlinkParser.send(bufferOut);
            }, 1000); // we should be able to define timeout in the future
        }*/

        //MAV.callbacks.push(obj);

        if (!data || requestExists) return false;

         
        mavlinkParser = new MAVLink10Processor();
        mavlinkParser.file = {};
        mavlinkParser.file.write = function(buf){
            len = buf.length;
            MAV.send_buffer = new ArrayBuffer(len);
            var bufView = new Uint8Array(MAV.send_buffer);
            for(var i=0;i<len;i++)
                bufView[i] = buf[i];
            serial.send(MAV.send_buffer, (info)=>{

                if (info.bytesSent == len) {
                    if (callback_sent){
                        callback_sent(info); //console.log("callback test", callback_sent, len);
                    }
                    rosflightIO.handle_callback = callback_msp;
                }
            });
        }
        
        mavlinkParser.send(bufferOut);

        return MAV.send_buffer;
    }
};


var rosflightIO  = {
    debug: 0,
    device: { 
        version: 0.0,
        show_stat: {},
        cid: 0,
        sensors: {acc:0,baro:0,gps:0,mag:0,sonar:0,gyro:0},
        time:0,
        errortime:0,
    },
    prev_status_ :  {
        heartbeat: 0,
        armed:  0,
        failsafe: 0,
        rc_override: 0,
        offboard: 0,
        error_code: 0,
        control_mode: 0
    },
    out_status: {

    },  
    attitude_msg:{
        attitude: {},
        angular_velocity:{}
    },
    imu_msg:{
        acc:{ x:0, y:0, z:0},
        gyro:{x:0, y:0, z:0}
    },
    out_msg:{
        rc: {
            values: []
        },
        servo: {
            buffer:{},
            values: []
        }
    },
    baro_msg: {

    },
    parameter_list: [
        
    ],
    alt_msg:{

    },
    error_msg:{

    },
    onOpen: ()=>{
        rosflightIO.device.errortime = Date.now();
    },
    onclose: ()=>{
        rosflightIO.device.show_stat = {};
    },
    toeuler: ()=>{
        const r2der =1; 
        const w = rosflightIO.attitude_msg.attitude.w;
        const x = rosflightIO.attitude_msg.attitude.x;
        const y = rosflightIO.attitude_msg.attitude.y;
        const z = rosflightIO.attitude_msg.attitude.z;

          var R =Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
          var P =Math.asin(2.0 * (w * y - z * x));
          var Y =Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    

        return {roll:(R )*r2der, pitch:(P)*r2der, yaw:Y};
        
    },
    payloat_to_number: (msg, start, type) =>{
        if(msg.payload==undefined)return 0;
        const buffer = new ArrayBuffer(16);
        const view = new DataView(buffer);
        for(var i=0;i<4;i++)
            view.setUint8(i, msg.payload[3-i]);
        if(msg.payload[msg.payload.length-1]==9)return msg.param_value;//return view.getFloat32(0);
        return view.getInt32(0);
    },
    payload_to_text: (msg, start, size)=>{
        var  uint8array = new Uint8Array(size);
        if(msg.payload != "undefined" && msg.payload[start] < 128){
            for(var i = 0;i<size;i++)
                uint8array[i] =  msg.payload[i+start];
        }
        var string = new TextDecoder().decode(uint8array); 
        return string.replace(/[^a-zA-Z0-9_ ]/g, "");
    },
    check_error_code: (error, chk, name)=>{

        if ((error & chk) == (chk))
        {
            if(rosflightIO.device.show_stat[chk] != name){
                if (chk & error){
                    console.log("ROSFLIGHT: Autopilot ERROR: ", name);
                    GUI.log("rosflight error "+  name);
                }else
                    console.log("ROSFLIGHT: Autopilot RECOVERED ERROR: ", name);
                
                rosflightIO.device.show_stat[chk] = name;
            }
        }

        if((Date.now() - rosflightIO.device.errortime) > 5000){
            rosflightIO.device.show_stat = [];
            rosflightIO.device.errortime = Date.now();
        }
    },
    handle_status_msg: (status_msg)=>{
        
        // armed state check
        if (rosflightIO.prev_status_.armed != status_msg.armed)
        {
            if (status_msg.armed)
            console.log("ROSFLIGHT: Autopilot ARMED");
            else
            console.log("ROSFLIGHT: Autopilot DISARMED");
            rosflightIO.prev_status_.armed = status_msg.armed;
        }

        // failsafe check
        if (rosflightIO.prev_status_.failsafe != status_msg.failsafe)
        {
            if (status_msg.failsafe)
            console.log("ROSFLIGHT: Autopilot FAILSAFE");
            else
            console.log("ROSFLIGHT: Autopilot FAILSAFE RECOVERED");
            rosflightIO.prev_status_.failsafe = status_msg.failsafe;
        }

        // rc override check
        if (rosflightIO.prev_status_.rc_override != status_msg.rc_override)
        {
            if (status_msg.rc_override)
            console.log("ROSFLIGHT: RC override active");
            else
            console.log("ROSFLIGHT: Returned to computer control");
            rosflightIO.prev_status_.rc_override = status_msg.rc_override;
        }

        // offboard control check
        if (rosflightIO.prev_status_.offboard != status_msg.offboard)
        {
            if (status_msg.offboard){
                console.log("ROSFLIGHT: Computer control active");
            }else
            console.log("ROSFLIGHT: Computer control lost");
            rosflightIO.prev_status_.offboard = status_msg.offboard;
        }

        // Print if got error code
        if (rosflightIO.prev_status_.error_code != status_msg.error_code)
        {
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_INVALID_MIXER, "Invalid mixer");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_IMU_NOT_RESPONDING, "IMU not responding");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_RC_LOST, "RC lost");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_UNHEALTHY_ESTIMATOR, "Unhealthy estimator");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_TIME_GOING_BACKWARDS, "Time going backwards");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_UNCALIBRATED_IMU, "Uncalibrated IMU");
            rosflightIO.check_error_code(status_msg.error_code, mavlink10.ROSFLIGHT_ERROR_BUFFER_OVERRUN, "Buffer Overrun");
            rosflightIO.prev_status_.error_code = status_msg.error_code;
        }

        // Print if change in control mode
        if (rosflightIO.prev_status_.control_mode != status_msg.control_mode)
        {
            var mode_string;
            switch (status_msg.control_mode)
            {
                case mavlink10.MODE_PASS_THROUGH:
                mode_string = "PASS_THROUGH";
                break;
                case mavlink10.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
                mode_string = "RATE";
                break;
                case mavlink10.MODE_ROLL_PITCH_YAWRATE_THROTTLE:
                mode_string = "ANGLE";
                break;
                default:
                mode_string = "UNKNOWN";
            }
            console.log("ROSFLIGHT: Autopilot now in " + mode_string +" mode");
            rosflightIO.prev_status_.control_mode = status_msg.control_mode;
        }

        rosflightIO.prev_status_ = status_msg;

        // Build the status message and send it
        rosflightIO.out_status.heartbeat = 1;
        rosflightIO.out_status.time = Date.now();
        rosflightIO.out_status.armed = status_msg.armed;
        rosflightIO.out_status.failsafe = status_msg.failsafe;
        rosflightIO.out_status.rc_override = status_msg.rc_override;
        rosflightIO.out_status.offboard = status_msg.offboard;
        rosflightIO.out_status.control_mode = status_msg.control_mode;
        rosflightIO.out_status.error_code = status_msg.error_code;
        rosflightIO.out_status.num_errors = status_msg.num_errors;
        rosflightIO.out_status.loop_time_us = status_msg.loop_time_us;
        rosflightIO.device.time = rosflightIO.out_status.time;

        
        ANALOG.last_received_timestamp = rosflightIO.attitude_msg.time; 

    },
    handle_command_ack_msg: (ack)=>{
        if (ack.success == mavlink10.ROSFLIGHT_CMD_SUCCESS)
        {
          console.log("ROSFLIGHT: MAVLink command "+ack.command+" Acknowledged");
        }
        else
        {
          console.log("ROSFLIGHT: MAVLink command "+ack.command+" Failed");
        }
    },
    handle_statustext_msg: (status)=>{
        var text = rosflightIO.payload_to_text(status, 1, 50);
        switch (status.severity)
        {
        case mavlink10.MAV_SEVERITY_EMERGENCY:
                console.log("[Autopilot EMERG]: ", text);
            break;
        case mavlink10.MAV_SEVERITY_ALERT:
                console.log("[Autopilot ALERT]: ", text); 
            break;
        case mavlink10.MAV_SEVERITY_CRITICAL: 
                console.log("[Autopilot CRITICAL]: ", text); 
            break;
        case mavlink10.MAV_SEVERITY_ERROR:
            console.log("[Autopilot ERR]: ", text); 
          break;
        case mavlink10.MAV_SEVERITY_WARNING:
            console.log("[Autopilot WARN]: ", text);
          break;
        case mavlink10.MAV_SEVERITY_NOTICE:
        case mavlink10.MAV_SEVERITY_INFO:
          console.log("[Autopilot INFO]: ", text);
          break;
        case mavlink10.MAV_SEVERITY_DEBUG:
            console.log("[Autopilot DEBUG]: ", text);
          break;
        }
        GUI.log(text);
    },
    handle_attitude_quaternion_msg: (attitude)=>{
        rosflightIO.attitude_msg.time = Date.now();
        rosflightIO.attitude_msg.attitude.w = attitude.q1;
        rosflightIO.attitude_msg.attitude.x = attitude.q2;
        rosflightIO.attitude_msg.attitude.y = attitude.q3;
        rosflightIO.attitude_msg.attitude.z = attitude.q4;
        rosflightIO.attitude_msg.angular_velocity.x = attitude.rollspeed;
        rosflightIO.attitude_msg.angular_velocity.y = attitude.pitchspeed;
        rosflightIO.attitude_msg.angular_velocity.z = attitude.yawspeed;
    },
    handle_small_imu_msg: (imu)=>{ 
        rosflightIO.imu_msg.time = Date.now();
        rosflightIO.imu_msg.frame_id = imu.frame_id;
        rosflightIO.imu_msg.acc.x = imu.xacc;
        rosflightIO.imu_msg.acc.y = imu.yacc;
        rosflightIO.imu_msg.acc.z = imu.zacc;
        rosflightIO.imu_msg.gyro.x = imu.xgyro;
        rosflightIO.imu_msg.gyro.y = imu.ygyro;
        rosflightIO.imu_msg.gyro.z = imu.zgyro;
        rosflightIO.imu_msg.timeus = rosflightIO.payloat_to_number(imu, 0, 6);
        //rosflightIO.imu_msg.orientation = attitude_quat_;
      
        rosflightIO.imu_msg.temperature = imu.temperature;
        rosflightIO.device.sensors.acc = 1;
        rosflightIO.device.sensors.gyro = 1;
    },
    handle_rosflight_output_raw_msg: (out)=>{
        rosflightIO.out_msg.servo.time = Date.now();
        rosflightIO.out_msg.servo.buffer = out;
        for (var i = 0; i < out.values.length; i++)
        {
          rosflightIO.out_msg.servo.values[i] = out.values[i];
        }
    },
    handle_rc_channels_raw_msg: (rc)=>{
        rosflightIO.out_msg.rc.time = Date.now();

        rosflightIO.out_msg.rc.values[0] = rc.chan1_raw;
        rosflightIO.out_msg.rc.values[1] = rc.chan2_raw;
        rosflightIO.out_msg.rc.values[2] = rc.chan3_raw;
        rosflightIO.out_msg.rc.values[3] = rc.chan4_raw;
        rosflightIO.out_msg.rc.values[4] = rc.chan5_raw;
        rosflightIO.out_msg.rc.values[5] = rc.chan6_raw;
        rosflightIO.out_msg.rc.values[6] = rc.chan7_raw;
        rosflightIO.out_msg.rc.values[7] = rc.chan8_raw;
        rosflightIO.out_msg.rc.rssi = rc.rssi;

    },
    handle_small_baro_msg: (baro)=>{
        rosflightIO.baro_msg.time = Date.now();
        rosflightIO.baro_msg.altitude = baro.altitude;
        rosflightIO.baro_msg.pressure = baro.pressure;
        rosflightIO.baro_msg.temperature = baro.temperature;
    },
    handle_small_range_msg: (ran)=>{
        rosflightIO.alt_msg.time = Date.now();
        rosflightIO.alt_msg.max_range = range.max_range;
        rosflightIO.alt_msg.min_range = range.min_range;
        rosflightIO.alt_msg.range = range.range;
    },
    handle_version_msg: (ver)=>{
        rosflightIO.device.version = parseFloat(ver.version) ;
        rosflightIO.onOpen();
    },
    handle_parameter: (msg)=>{
        var object = {
          "param_id": rosflightIO.payload_to_text(msg, 8, 16),
          "param_index": msg.param_index,
          "param_value": rosflightIO.payloat_to_number(msg, 0, msg.param_type),
          "param_type": msg.payload[msg.payload.length-1],
        };
        if(rosflightIO.parameter_list[msg.param_index]=="undefined")
        rosflightIO.parameter_list.push(object);
        else {
            rosflightIO.parameter_list[msg.param_index] = object;
        }
//console.log(msg);

        SERIAL_CONFIG.mspBaudRate = rosflightIO.handle_get_param("BAUD_RATE");
        MIXER_CONFIG.mixer = rosflightIO.handle_get_param("MIXER");  
        MIXER_CONFIG.mixer = MIXER_CONFIG.mixer > mixerList.length ? 0 : MIXER_CONFIG.mixer; 
        MIXER_CONFIG.reverseMotorDir = 0; 
        
    },
    handle_hard_error_msg: (error)=>{
        console.log("Hard fault detected, with error code %u. The flight controller has rebooted.", error.error_code);
        console.log("Hard fault was at: 0x%x", error.pc);
        if (error.doRearm)
        {
          console.log("The firmware has rearmed itself.");
        }
        console.log("The flight controller has rebooted %u time%s.", error.reset_count, error.reset_count > 1 ? "s" : "");
        rosflightIO.error_msg.error_message = "A firmware error has caused the flight controller to reboot.";
        rosflightIO.error_msg.error_code = error.error_code;
        rosflightIO.error_msg.reset_count = error.reset_count;
        rosflightIO.error_msg.rearm = error.doRearm;
        rosflightIO.error_msg.pc = error.pc;
    },
    handle_message: (msg)=>{
        if(rosflightIO.device.cid !=serial.connectionId){
            rosflightIO.device.cid = serial.connectionId;
            rosflightIO.onclose();
        }

        switch (msg.id)
        {
        case mavlink10.MAVLINK_MSG_ID_HEARTBEAT:
            rosflightIO.out_status.heartbeat = 2;
            //conected
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_STATUS:
            rosflightIO.handle_status_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK:
            rosflightIO.handle_command_ack_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_STATUSTEXT:
            rosflightIO.handle_statustext_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            rosflightIO.handle_attitude_quaternion_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_SMALL_IMU:
            rosflightIO.handle_small_imu_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_SMALL_MAG:
            //handle_small_mag_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_OUTPUT_RAW:
            rosflightIO.handle_rosflight_output_raw_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_RC_CHANNELS:
            rosflightIO.handle_rc_channels_raw_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_DIFF_PRESSURE:
            //handle_diff_pressure_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_NAMED_VALUE_INT:
            //handle_named_value_int_msg(msg);
                console.log(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
            //handle_named_value_float_msg(msg);
                console.log(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT:
            //handle_named_command_struct_msg(msg);
                console.log(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_SMALL_BARO:
            rosflightIO.handle_small_baro_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_SMALL_RANGE:
            rosflightIO.handle_small_range_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_GNSS:
            //handle_rosflight_gnss_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_GNSS_FULL:
            //handle_rosflight_gnss_full_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_VERSION:
            rosflightIO.handle_version_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_PARAM_VALUE:
            rosflightIO.handle_parameter(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_TIMESYNC:
            // silently ignore (handled elsewhere)
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR:
            handle_hard_error_msg(msg);
            break;
        case mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS:
            //handle_battery_status_msg(msg);
                console.log(msg);
            break;
        default:
            //console.log("rosflight_io: Got unhandled mavlink message ID ", msg.msgid);
            break;
        }
        
        rosflightIO.handle_msg_callback(msg);
    }, 
    handle_msg_callback_id: 0,
    handle_msg_callback: (msg)=>{
        if(rosflightIO.handle_callback && rosflightIO.handle_msg_callback_id == msg.id){
            rosflightIO.handle_callback(msg);
            rosflightIO.handle_callback = false;//one time run callback
        } 
        
        if(rosflightIO.debug)console.log(msg);
    },
    handle_callback: false,
    setArmingEnabled: (doEnable, disableRunawayTakeoffPrevention, onCompleteCallback)=>{
        if (CONFIG.armingDisabled === doEnable || CONFIG.runawayTakeoffPreventionDisabled !== disableRunawayTakeoffPrevention) {
    
            CONFIG.armingDisabled = !doEnable;
            CONFIG.runawayTakeoffPreventionDisabled = disableRunawayTakeoffPrevention;
    
            //MAV.send_message(MSPCodes.MSP_ARMING_DISABLE, mspHelper.crunch(MSPCodes.MSP_ARMING_DISABLE), false, function () {
                if (doEnable) {
                    GUI.log(i18n.getMessage('armingEnabled'));
                    if (disableRunawayTakeoffPrevention) {
                        GUI.log(i18n.getMessage('runawayTakeoffPreventionDisabled'));
                    } else {
                        GUI.log(i18n.getMessage('runawayTakeoffPreventionEnabled'));
                    }
                } else {
                    GUI.log(i18n.getMessage('armingDisabled'));
                }
    
                if (onCompleteCallback) {
                    onCompleteCallback();
                }
            //});
        } else {
            if (onCompleteCallback) {
                onCompleteCallback();
            }
        } 
    },
    handle_parameter_set: (param_name, value)=>{
        var index = 0, type = mavlink10.MAV_PARAM_TYPE_INT32;
        for (var key in rosflightIO.parameter_list){
        
            if(rosflightIO.parameter_list[key].param_id==param_name){
                if(rosflightIO.parameter_list[key].param_id != "undefined"){
                    index = key;
                    return {param_id:parseInt(index), param_value:value, param_type:rosflightIO.parameter_list[key].param_type};;
                }
            }
        }
        return false;
    },
    save_parameter: (cb)=>{
        rosflightIO.handle_msg_callback_id = mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK;
        MAV.send_message(mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD, mavlink10.ROSFLIGHT_CMD_WRITE_PARAMS, 0, ()=>{
            if(cb)cb();
        }, 1);
    },
    handle_get_param: (name)=>{
        if(rosflightIO.parameter_list.length<113) return 0;
        var id = rosflightIO.handle_parameter_set(name).param_id;
        return rosflightIO.parameter_list[id].param_value;
    },
    update_imu: ()=>{
        SENSOR_DATA.accelerometer[0] = rosflightIO.imu_msg.acc.x / 9.8066;
        SENSOR_DATA.accelerometer[1] = rosflightIO.imu_msg.acc.y / 9.8066;
        SENSOR_DATA.accelerometer[2] = rosflightIO.imu_msg.acc.z / 9.8066;
        SENSOR_DATA.gyroscope[0] = rosflightIO.imu_msg.gyro.x * 57.2957795;
        SENSOR_DATA.gyroscope[1] = rosflightIO.imu_msg.gyro.y * 57.2957795;
        SENSOR_DATA.gyroscope[2] = rosflightIO.imu_msg.gyro.z * 57.2957795;
        
        var euler = rosflightIO.toeuler();
        SENSOR_DATA.kinematics[0] = euler.roll;
        SENSOR_DATA.kinematics[1] = euler.pitch;
        SENSOR_DATA.kinematics[2] = euler.yaw;
    }
};

//serial.onReceive.addListener(MAV.read); 
///DEBUG

function con(){
    serial.connect(String($('div#port-picker #port').val()), {bitrate: 115200}, onOpens); 
    function onOpens(info){
            serial.onReceive.addListener(MAV.read);
        console.log(info); 
    }
};

function ros_reboot(){
    mavlinkParser.send(new mavlink10.messages.rosflight_cmd(mavlink10.ROSFLIGHT_CMD_REBOOT));
}


function invSqrt(x) {
    var halfx = 0.5 * x;
    var y = x;
    var i = y;
    i = 0x5f3759df - (i >> 1);
    y = i;
    y = y * (1.5 - (halfx * y * y));
    y = y * (1.5 - (halfx * y * y));
    return y;
}

    var q0 = 1,q1 = 0,q2 = 0,q3 = 0;

function imu_mahonyAHRSupdate6DO() {

    var ax = rosflightIO.imu_msg.acc.z;
    var ay = rosflightIO.imu_msg.acc.y;
    var az = rosflightIO.imu_msg.acc.x;
    var gx = rosflightIO.imu_msg.gyro.z;
    var gy = rosflightIO.imu_msg.gyro.y;
    var gz = rosflightIO.imu_msg.gyro.x;
    var dt  = rosflightIO.imu_msg.timeus / 1000000.0;
    dt = 0.006;
    var betas = 1.5;
    var recipNorm;
    var s0, s1, s2, s3;
    var qDot1, qDot2, qDot3, qDot4;
    var _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
        q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    //gx *= 0.0174533;
    //gy *= 0.0174533;
    //gz *= 0.0174533;

    // Rate o change o quaternion rom gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    // Compute eedback only i accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (1) {

      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0 * q0;
      _2q1 = 2.0 * q1;
      _2q2 = 2.0 * q2;
      _2q3 = 2.0 * q3;
      _4q0 = 4.0 * q0;
      _4q1 = 4.0 * q1;
      _4q2 = 4.0 * q2;
      _8q1 = 8.0 * q1;
      _8q2 = 8.0 * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 +
           _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
           _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply eedback step
      qDot1 -= betas * s0;
      qDot2 -= betas * s1;
      qDot3 -= betas * s2;
      qDot4 -= betas * s3;
    }

    // Integrate rate o change o quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    return {w:q0,x:q1,y:q2,z:q3};
}

function sensor2_euler(){
    var quad = imu_mahonyAHRSupdate6DO();
    var w = quad.w;
    var x = quad.x;
    var y = quad.y;
    var z = quad.z;
    SENSOR_DATA.kinematics[0] = Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    SENSOR_DATA.kinematics[1] = Math.asin(2.0 * (w * y - z * x));
    SENSOR_DATA.kinematics[2] = Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}