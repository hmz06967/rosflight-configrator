'use strict';

TABS.setup = {
    yaw_fix: 0.0
};

TABS.map = function(x, in_min, in_max, out_min, out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

TABS.setup.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'setup') {
        GUI.active_tab = 'setup';
    }

    function load_status() {
        //MSP.send_message(MSPCodes.MSP_STATUS, false, false, load_mixer_config);
        load_mixer_config();
    }

    function load_mixer_config() {
        //MSP.send_message(MSPCodes.MSP_MIXER_CONFIG, false, false, load_html);
        MIXER_CONFIG.mixer = 2; //rosflightIO.handle_get_param("MIXER");  
        //MIXER_CONFIG.mixer = (MIXER_CONFIG.mixer > 14 || MIXER_CONFIG.mixer < 0) ? 0 : MIXER_CONFIG.mixer;
        //MIXER_CONFIG.reverseMotorDir = rosflightIO.handle_get_param("ELEVATOR_REV");    
        load_html();
    }

    function load_html() {
        $('#content').load("./tabs/setup.html", process_html);
    }

    //MSP.send_message(MSPCodes.MSP_ACC_TRIM, false, false, load_status);

    load_status();

    function process_html() {
        // translate to user-selected language
        i18n.localizePage();

        if (semver.lt(CONFIG.apiVersion, CONFIGURATOR.backupRestoreMinApiVersionAccepted)) {
            $('#content .backup').addClass('disabled');
            $('#content .restore').addClass('disabled');

            GUI.log(i18n.getMessage('initialSetupBackupAndRestoreApiVersion', [CONFIG.apiVersion, CONFIGURATOR.backupRestoreMinApiVersionAccepted]));
        }

        // initialize 3D Model
        self.initModel();

		// set roll in interactive block
        $('span.roll').text(i18n.getMessage('initialSetupAttitude', [0]));
		// set pitch in interactive block
        $('span.pitch').text(i18n.getMessage('initialSetupAttitude', [0]));
        // set heading in interactive block
        $('span.heading').text(i18n.getMessage('initialSetupAttitude', [0]));

        // check if we have accelerometer and magnetometer
        if (!have_sensor(CONFIG.activeSensors, 'acc')) {
            $('a.calibrateAccel').addClass('disabled');
            $('default_btn').addClass('disabled');
        }

        if (!have_sensor(CONFIG.activeSensors, 'mag')) {
            $('a.calibrateMag').addClass('disabled');
            $('default_btn').addClass('disabled');
        }

        self.initializeInstruments();

        $('#arming-disable-flag-row').attr('title', i18n.getMessage('initialSetupArmingDisableFlagsTooltip'));

        if (semver.gte(CONFIG.apiVersion, "1.40.0")) {
            if (isExpertModeEnabled()) {
                $('.initialSetupRebootBootloader').show();
            } else {
                $('.initialSetupRebootBootloader').hide();
            }

            $('a.rebootBootloader').click(function () {
                var buffer = [];
                buffer.push(1);
                MSP.send_message(MSPCodes.MSP_SET_REBOOT, buffer, false);
            });
        } else {
            $('.initialSetupRebootBootloader').hide();
        }

        // UI Hooks
        $('a.calibrateAccel').click(function () {
            var self = $(this);

            if (!self.hasClass('calibrating')) {
                self.addClass('calibrating');

                // During this period MCU won't be able to process any serial commands because its locked in a for/while loop
                // until this operation finishes, sending more commands through data_poll() will result in serial buffer overflow
                GUI.interval_pause('setup_data_pull');
                //MSP.send_message(MSPCodes.MSP_ACC_CALIBRATION, false, false, function () { });
                MAV.send_message(mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD, mavlink10.ROSFLIGHT_CMD_ACCEL_CALIBRATION, (i)=>{
                    GUI.log(i18n.getMessage('initialSetupAccelCalibStarted'));
                    $('#accel_calib_running').show();
                    $('#accel_calib_rest').hide();      
                }, 0, 1);
                GUI.timeout_add('button_reset', function () {
                    GUI.interval_resume('setup_data_pull');

                    GUI.log(i18n.getMessage('initialSetupAccelCalibEnded'));
                    self.removeClass('calibrating');
                    $('#accel_calib_running').hide();
                    $('#accel_calib_rest').show();
                }, 2000);
            }
        });

        $('a.calibrateMag').click(function () {
            var self = $(this);

            if (!self.hasClass('calibrating') && !self.hasClass('disabled')) {
                self.addClass('calibrating');

                MSP.send_message(MSPCodes.MSP_MAG_CALIBRATION, false, false, function () {
                    GUI.log(i18n.getMessage('initialSetupMagCalibStarted'));
                    $('#mag_calib_running').show();
                    $('#mag_calib_rest').hide();
                });

                GUI.timeout_add('button_reset', function () {
                    GUI.log(i18n.getMessage('initialSetupMagCalibEnded'));
                    self.removeClass('calibrating');
                    $('#mag_calib_running').hide();
                    $('#mag_calib_rest').show();
                }, 30000);
            }
        });

        var dialogConfirmReset = $('.dialogConfirmReset')[0];

        $('a.resetSettings').click(function () {
            dialogConfirmReset.showModal();
        });

        $('.dialogConfirmReset-cancelbtn').click(function() {
            dialogConfirmReset.close();
        });

        $('.dialogConfirmReset-confirmbtn').click(function() {
            dialogConfirmReset.close();

            rosflightIO.handle_msg_callback_id = mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK; 
            MAV.send_message(mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD, mavlink10.ROSFLIGHT_CMD_SET_PARAM_DEFAULTS, 0, (info)=>{
               GUI.log(i18n.getMessage('initialSetupSettingsRestored'));

                GUI.tab_switch_cleanup(function () {
                    TABS.setup.initialize();
                }); 

                rosflightIO.save_parameter(); 
            }, 0);
            
            //MSP.send_message(MSPCodes.MSP_RESET_CONF, false, false, function () { });
        });

        // display current yaw fix value (important during tab re-initialization)
        $('div#interactive_block > a.reset').text(i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));

        // reset yaw button hook
        $('div#interactive_block > a.reset').click(function () {
            self.yaw_fix = SENSOR_DATA.kinematics[2] * - 1.0;
            $(this).text(i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));

            console.log('YAW reset to 0 deg, fix: ' + self.yaw_fix + ' deg');
        });

        $('#content .backup').click(function () {
            if ($(this).hasClass('disabled')) {
                return;
            }

            configuration_backup(function () {
                GUI.log(i18n.getMessage('initialSetupBackupSuccess'));
            });
        });

        $('#content .restore').click(function () {
            if ($(this).hasClass('disabled')) {
                return;
            }

            configuration_restore(function () {
                // get latest settings
                TABS.setup.initialize();

                GUI.log(i18n.getMessage('initialSetupRestoreSuccess'));
            });
        });

        // cached elements
        var bat_voltage_e = $('.bat-voltage'),
            bat_mah_drawn_e = $('.bat-mah-drawn'),
            bat_mah_drawing_e = $('.bat-mah-drawing'),
            rssi_e = $('.rssi'),
            arming_disable_flags_e = $('.arming-disable-flags'),
            gpsFix_e = $('.gpsFix'),
            gpsSats_e = $('.gpsSats'),
            gpsLat_e = $('.gpsLat'),
            gpsLon_e = $('.gpsLon'),
            roll_e = $('dd.roll'),
            pitch_e = $('dd.pitch'),
            heading_e = $('dd.heading');

        if (semver.lt(CONFIG.apiVersion, "1.36.0")) {
            //arming_disable_flags_e.hide();
        }

        function get_slow_data() {
            //MSP.send_message(MSPCodes.MSP_STATUS, false, false, function() {});
            MAV.send_message(mavlink10.MAVLINK_MSG_ID_HEARTBEAT, 1, (i)=>{
                var error = rosflightIO.device.show_stat[rosflightIO.out_status.error_code];
                var armingString = '';
                if(error){
                    CONFIG.armingDisableFlags = error;
                    
                    //CONFIG.armingDisableFlags = rosflightIO.out_status.armed;
                    if (CONFIG.armingDisableFlags == 0) {
                        armingString = i18n.getMessage('initialSetupArmingAllowed');
                    } else {
                        /*var flagIndicies = [];
                        for (var i = 0; i < 32; i++) {
                            if (CONFIG.armingDisableFlags & (1 << i)) {
                              flagIndicies.push(i + 1);
                            }
                        }*/
                        armingString = error;
                    }
                   
                }
                arming_disable_flags_e.text(armingString);
            },0);
            
            //MSP.send_message(MSPCodes.MSP_ANALOG, false, false, function () { });
            
                bat_voltage_e.text(i18n.getMessage('initialSetupBatteryValue', [ANALOG.voltage]));
                bat_mah_drawn_e.text(i18n.getMessage('initialSetupBatteryMahValue', [ANALOG.mAhdrawn]));
                bat_mah_drawing_e.text(i18n.getMessage('initialSetupBatteryAValue', [ANALOG.amperage.toFixed(2)]));
                rssi_e.text(i18n.getMessage('initialSetupRSSIValue', [((ANALOG.rssi / 1023) * 100).toFixed(0)]));
            
            if (have_sensor(CONFIG.activeSensors, 'gps')) {
                MSP.send_message(MSPCodes.MSP_RAW_GPS, false, false, function () {
                    gpsFix_e.html((GPS_DATA.fix) ? i18n.getMessage('gpsFixTrue') : i18n.getMessage('gpsFixFalse'));
                    gpsSats_e.text(GPS_DATA.numSat);
                    gpsLat_e.text((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
                    gpsLon_e.text((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');
                });
            }


        }

        function get_fast_data() {
            /*MSP.send_message(MSPCodes.MSP_ATTITUDE, false, false, function () {

            });
*/ 
            rosflightIO.update_imu();
            //ensor2_euler();
            
            roll_e.text(i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[0].toFixed(3)]));
            pitch_e.text(i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[1].toFixed(3)]));
            heading_e.text(i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[2].toFixed(3)]));

            self.renderModel();
            self.updateInstruments();
            
        }

        GUI.interval_add('setup_data_pull_fast', get_fast_data, 33, true); // 30 fps
        GUI.interval_add('setup_data_pull_slow', get_slow_data, 250, true); // 4 fps

        GUI.content_ready(callback);
    }
};

TABS.setup.initializeInstruments = function() {
    var options = {size:90, showBox : false, img_directory: 'images/flightindicators/'};
    var attitude = $.flightIndicator('#attitude', 'attitude', options);
    var heading = $.flightIndicator('#heading', 'heading', options);

    this.updateInstruments = function() {
        attitude.setRoll(SENSOR_DATA.kinematics[0].toFixed(3));
        attitude.setPitch(SENSOR_DATA.kinematics[1].toFixed(3));
        heading.setHeading(SENSOR_DATA.kinematics[2].toFixed(3));
    };
};

TABS.setup.initModel = function () {
    this.model = new Model($('.model-and-info #canvas_wrapper'), $('.model-and-info #canvas'));

    $(window).on('resize', $.proxy(this.model.resize, this.model));
};

TABS.setup.renderModel = function () {
    var x = (SENSOR_DATA.kinematics[1] * -1.0) * 0.017453292519943295,
        y = ((SENSOR_DATA.kinematics[2] * -1.0) - this.yaw_fix) * 0.017453292519943295,
        z = (SENSOR_DATA.kinematics[0] * -1.0) * 0.017453292519943295;

    this.model.rotateTo(x, y, z);
};

TABS.setup.cleanup = function (callback) {
    if (this.model) {
        $(window).off('resize', $.proxy(this.model.resize, this.model));
    }

    if (callback) callback();
};
