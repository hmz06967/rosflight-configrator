'use strict';
var mspHelper;

var connectionTimestamp;

function initializeSerialBackend() {

    GUI.updateManualPortVisibility = function(){
        var selected_port = $('div#port-picker #port option:selected');
        if (selected_port.data().isManual) {
            $('#port-override-option').show();
        }
        else {
            $('#port-override-option').hide();
        }
        if (selected_port.data().isDFU) {
            $('select#baud').hide();
        }
        else {
            $('select#baud').show();
        }
    };

    GUI.updateManualPortVisibility();

    $('#port-override').change(function () {
        chrome.storage.local.set({'portOverride': $('#port-override').val()});
    });

    chrome.storage.local.get('portOverride', function (data) {
        $('#port-override').val(data.portOverride);
    });

    $('div#port-picker #port').change(function (target) {
        GUI.updateManualPortVisibility();
    });

    $('div.connect_controls a.connect').click(function () {
        
        if (GUI.connect_lock != true) { // GUI control overrides the user control

            var thisElement = $(this);
            var clicks = thisElement.data('clicks');

            var toggleStatus = function() {
                thisElement.data("clicks", !clicks);
            };

            GUI.configuration_loaded = false;

            var selected_baud = parseInt($('div#port-picker #baud').val());
            var selected_port = $('div#port-picker #port option:selected').data().isManual ?
                    $('#port-override').val() :
                    String($('div#port-picker #port').val());
            if (selected_port === 'DFU') {
                GUI.log(i18n.getMessage('dfu_connect_message'));
            }
            else if (selected_port != '0') {
                if (!clicks) {
                    console.log('Connecting to: ' + selected_port);
                    GUI.connecting_to = selected_port;

                    // lock port select & baud while we are connecting / connected
                    $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', true);
                    $('div.connect_controls a.connect_state').text(i18n.getMessage('connecting'));

                    serial.connect(selected_port, {bitrate: selected_baud}, onOpen);

                    toggleStatus();
                } else {
                    GUI.timeout_kill_all();
                    GUI.interval_kill_all();
                    GUI.tab_switch_cleanup();
                    GUI.tab_switch_in_progress = false;

                    function onFinishCallback() {
                        finishClose(toggleStatus);
                    }

                    onFinishCallback();
                    console.log("disconnect");
                    
                    //mspHelper.setArmingEnabled(true, false, onFinishCallback);
                }
            }
       }
    });

    // auto-connect
    chrome.storage.local.get('auto_connect', function (result) {
        if (result.auto_connect === 'undefined' || result.auto_connect) {
            // default or enabled by user
            GUI.auto_connect = true;

            $('input.auto_connect').prop('checked', true);
            $('input.auto_connect, span.auto_connect').prop('title', i18n.getMessage('autoConnectEnabled'));

            $('select#baud').val(115200).prop('disabled', true);
        } else {
            // disabled by user
            GUI.auto_connect = false;

            $('input.auto_connect').prop('checked', false);
            $('input.auto_connect, span.auto_connect').prop('title', i18n.getMessage('autoConnectDisabled'));
        }

        // bind UI hook to auto-connect checkbos
        $('input.auto_connect').change(function () {
            GUI.auto_connect = $(this).is(':checked');

            // update title/tooltip
            if (GUI.auto_connect) {
                $('input.auto_connect, span.auto_connect').prop('title', i18n.getMessage('autoConnectEnabled'));

                $('select#baud').val(115200).prop('disabled', true);
            } else {
                $('input.auto_connect, span.auto_connect').prop('title', i18n.getMessage('autoConnectDisabled'));

                if (!GUI.connected_to && !GUI.connecting_to) $('select#baud').prop('disabled', false);
            }

            chrome.storage.local.set({'auto_connect': GUI.auto_connect});
        });
    });

    PortHandler.initialize();
    PortUsage.initialize();
}

function finishClose(finishedCallback) {
    var wasConnected = CONFIGURATOR.connectionValid;

    analytics.sendEvent(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'Disconnected');
    if (connectionTimestamp) {
        var connectedTime = Date.now() - connectionTimestamp;
        analytics.sendTiming(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'Connected', connectedTime);

        connectedTime = undefined;
    }
    analytics.resetFlightControllerData();

    serial.disconnect(onClosed);

    //MSP.disconnect_cleanup();
    PortUsage.reset();

    GUI.connected_to = false;
    GUI.allowedTabs = GUI.defaultAllowedTabsWhenDisconnected.slice();
    // Reset various UI elements
    $('span.i2c-error').text(0);
    $('span.cycle-time').text(0);
    //if (semver.gte(CONFIG.apiVersion, "1.20.0"))
      //  $('span.cpu-load').text('');

    // unlock port select & baud
    $('div#port-picker #port').prop('disabled', false);
    if (!GUI.auto_connect) $('div#port-picker #baud').prop('disabled', false);

    // reset connect / disconnect button
    $('div.connect_controls a.connect').removeClass('active');
    $('div.connect_controls a.connect_state').text(i18n.getMessage('connect'));

    // reset active sensor indicators
    sensor_status(0);

    if (wasConnected) {
        // detach listeners and remove element data
        $('#content').empty();
    }

    $('#tabs .tab_landing a').click();

    finishedCallback();
}


function onOpen(openInfo) {
    if (openInfo) {
        // update connected_to
        GUI.connected_to = GUI.connecting_to;

        // reset connecting_to
        GUI.connecting_to = false;

        GUI.log(i18n.getMessage('serialPortOpened', [openInfo.connectionId]));

        // save selected port with chrome.storage if the port differs
        chrome.storage.local.get('last_used_port', function (result) {
            if (result.last_used_port) {
                if (result.last_used_port != GUI.connected_to) {
                    // last used port doesn't match the one found in local db, we will store the new one
                    chrome.storage.local.set({'last_used_port': GUI.connected_to});
                }
            } else {
                // variable isn't stored yet, saving
                chrome.storage.local.set({'last_used_port': GUI.connected_to});
            }
        });

        serial.onReceive.addListener(MAV.read); 
        //serial.onReceive.addListener(read_serial);

        // disconnect after 10 seconds with error if we don't get IDENT data
        GUI.timeout_add('connecting', function () {
            if (!CONFIGURATOR.connectionValid) {
                GUI.log(i18n.getMessage('noConfigurationReceived'));

                $('div.connect_controls a.connect').click(); // disconnect
            }
        }, 10000);

        FC.resetState();

        function connected_ros(){
            MAV.send_message(mavlink10.MAVLINK_MSG_ID_PARAM_REQUEST_LIST, 1, false, ()=>{}, true); 

            /**/
            /*MAV.send_message(mavlink10.MAVLINK_MSG_ID_OFFBOARD_CONTROL, {mode:mavlink10.MODE_ROLL_PITCH_YAWRATE_THROTTLE, x:1000.0, y:1000.0, z:1000.0, F:1000.0}, false, ()=> {}, 1);*/
            
            connectionTimestamp = Date.now();
            CONFIG.armingDisabled  = false;
            CONFIG.apiVersion = "1.5.0";
            RC.active_channels = rosflightIO.out_msg.rc.values.length;
            AUX_CONFIG = ["ARM", "ATTITUDE OVERRIDE", "THROTTLE OVERRIDE", "ATTITUDE TYPE", "FAILSAFE"];
            AUX_CONFIG_IDS = [0,1,2,3,4];
            CONFIG.mode = 0;
            
            finishOpen();
            //CONFIGURATOR.connectionValid = true;
            GUI.log("Mavlink javascript v1.0 rosflight");    
        }
        
        function connect_ros(){
            MAV.send_message(mavlink10.MAVLINK_MSG_ID_HEARTBEAT, 1, (i)=>{
                console.log("connection rosflight device..");
                if(rosflightIO.device.version > 0){
                    CONFIG.flightControllerVersion = rosflightIO.device.version.toString();
                    connected_ros();
                }else{
                    rosflightIO.device.version = 1;
                    rosflightIO.handle_msg_callback_id = mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_VERSION;
                    MAV.send_message( mavlink10.MAVLINK_MSG_ID_ROSFLIGHT_CMD,  mavlink10.ROSFLIGHT_CMD_SEND_VERSION, 0, (ver)=>{
                        CONFIG.flightControllerVersion = ver.version.replace(/[^0-9_.]/g, ""); 
                        console.log("rosflight version ", CONFIG.flightControllerVersion); 
                        
                        if(parseInt(CONFIG.flightControllerVersion) > 0){
                            connected_ros();
                        }else{
                            var dialog = $('.dialogConnectWarning')[0];
                            $('.dialogConnectWarning-content').html(i18n.getMessage('firmwareVersionNotSupported', [CONFIGURATOR.apiVersionAccepted]));
                            $('.dialogConnectWarning-closebtn').click(function() {
                                dialog.close();
                            });
                            
                            dialog.showModal();
                            connectCli();
                        }
                        //console.log(ver);
                        //clearTimeout(ros);
                    }, 1);      
                }
            }, 0, true);
        }

        connect_ros();
        
        /*MSP.listen(update_packet_error);
        mspHelper = new MspHelper();
        MSP.listen(mspHelper.process_data.bind(mspHelper));

        // request configuration data
        MSP.send_message(MSPCodes.MSP_API_VERSION, false, false, function () {
            analytics.setFlightControllerData(analytics.DATA.API_VERSION, CONFIG.apiVersion);

            GUI.log(i18n.getMessage('apiVersionReceived', [CONFIG.apiVersion]));

            if (semver.gte(CONFIG.apiVersion, CONFIGURATOR.apiVersionAccepted)) {

                MSP.send_message(MSPCodes.MSP_FC_VARIANT, false, false, function () {
                    analytics.setFlightControllerData(analytics.DATA.FIRMWARE_TYPE, CONFIG.flightControllerIdentifier);
                    if (CONFIG.flightControllerIdentifier === 'BTFL') {
                        MSP.send_message(MSPCodes.MSP_FC_VERSION, false, false, function () {
                             analytics.setFlightControllerData(analytics.DATA.FIRMWARE_VERSION, CONFIG.flightControllerVersion);

                            GUI.log(i18n.getMessage('fcInfoReceived', [CONFIG.flightControllerIdentifier, CONFIG.flightControllerVersion]));
                            updateStatusBarVersion(CONFIG.flightControllerVersion, CONFIG.flightControllerIdentifier);
                            updateTopBarVersion(CONFIG.flightControllerVersion, CONFIG.flightControllerIdentifier);

                            MSP.send_message(MSPCodes.MSP_BUILD_INFO, false, false, function () {

                                GUI.log(i18n.getMessage('buildInfoReceived', [CONFIG.buildInfo]));

                                MSP.send_message(MSPCodes.MSP_BOARD_INFO, false, false, function () {
                                    analytics.setFlightControllerData(analytics.DATA.BOARD_TYPE, CONFIG.boardIdentifier);

                                    GUI.log(i18n.getMessage('boardInfoReceived', [CONFIG.boardIdentifier, CONFIG.boardVersion]));
                                    updateStatusBarVersion(CONFIG.flightControllerVersion, CONFIG.flightControllerIdentifier, CONFIG.boardIdentifier);
                                    updateTopBarVersion(CONFIG.flightControllerVersion, CONFIG.flightControllerIdentifier, CONFIG.boardIdentifier);

                                    MSP.send_message(MSPCodes.MSP_UID, false, false, function () {
                                        var uniqueDeviceIdentifier = CONFIG.uid[0].toString(16) + CONFIG.uid[1].toString(16) + CONFIG.uid[2].toString(16);

                                        analytics.setFlightControllerData(analytics.DATA.MCU_ID, objectHash.sha1(uniqueDeviceIdentifier));
                                        analytics.sendEvent(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'Connected');
                                        connectionTimestamp = Date.now();
                                        GUI.log(i18n.getMessage('uniqueDeviceIdReceived', [uniqueDeviceIdentifier]));

                                        if (semver.gte(CONFIG.apiVersion, "1.20.0")) {
                                            MSP.send_message(MSPCodes.MSP_NAME, false, false, function () {
                                                GUI.log(i18n.getMessage('craftNameReceived', [CONFIG.name]));

                                                CONFIG.armingDisabled = false;
                                                mspHelper.setArmingEnabled(false, false, setRtc);
                                            });
                                        } else {
                                            setRtc();
                                        }
                                    });
                                });
                            });
                        });
                    } else {
                        analytics.sendEvent(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'ConnectionRefused');

                        var dialog = $('.dialogConnectWarning')[0];

                        $('.dialogConnectWarning-content').html(i18n.getMessage('firmwareTypeNotSupported'));

                        $('.dialogConnectWarning-closebtn').click(function() {
                            dialog.close();
                        });

                        dialog.showModal();

                        connectCli();
                    }
                });
            } else {
                analytics.sendEvent(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'ConnectionRefused');

                var dialog = $('.dialogConnectWarning')[0];

                $('.dialogConnectWarning-content').html(i18n.getMessage('firmwareVersionNotSupported', [CONFIGURATOR.apiVersionAccepted]));

                $('.dialogConnectWarning-closebtn').click(function() {
                    dialog.close();
                });

                dialog.showModal();

                connectCli();
            }
        });*/
        
    } else {
        analytics.sendEvent(analytics.EVENT_CATEGORIES.FLIGHT_CONTROLLER, 'SerialPortFailed');

        console.log('Failed to open serial port');
        GUI.log(i18n.getMessage('serialPortOpenFail'));

        $('div#connectbutton a.connect_state').text(i18n.getMessage('connect'));
        $('div#connectbutton a.connect').removeClass('active');

        // unlock port select & baud
        $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', false);

        // reset data
        $('div#connectbutton a.connect').data("clicks", false);
    }
}

function setRtc() {
    if (semver.gte(CONFIG.apiVersion, "1.37.0")) {
        MSP.send_message(MSPCodes.MSP_SET_RTC, mspHelper.crunch(MSPCodes.MSP_SET_RTC), false, finishOpen);
    } else {
        finishOpen();
    }
}

function finishOpen() {
    CONFIGURATOR.connectionValid = true;
    GUI.allowedTabs = GUI.defaultAllowedFCTabsWhenConnected.slice();
    if (semver.lt(CONFIG.apiVersion, "1.4.0")) {
        GUI.allowedTabs.splice(GUI.allowedTabs.indexOf('led_strip'), 1);
    }

    onConnect();

    GUI.selectDefaultTabWhenConnected();
}

function connectCli() {
    CONFIGURATOR.connectionValid = true; // making it possible to open the CLI tab
    GUI.allowedTabs = ['cli'];
    onConnect();
    $('#tabs .tab_cli a').click();
}

function onConnect() {
    GUI.timeout_remove('connecting'); // kill connecting timer
    $('div#connectbutton a.connect_state').text(i18n.getMessage('disconnect')).addClass('active');
    $('div#connectbutton a.connect').addClass('active');

    $('#tabs ul.mode-disconnected').hide();
    $('#tabs ul.mode-connected-cli').show();


    // show only appropriate tabs
    $('#tabs ul.mode-connected li').hide();
    $('#tabs ul.mode-connected li').filter(function (index) {
        var classes = $(this).attr("class").split(/\s+/);
        var found = false;
        $.each(GUI.allowedTabs, function (index, value) {
            var tabName = "tab_" + value;
            if ($.inArray(tabName, classes) >= 0) {
                found = true;
            }
        });

        if (CONFIG.boardType == 0) {
            if (classes.indexOf("osd-required") >= 0) {
                found = false;
            }
        }

        return found;
    }).show();

    if (CONFIG.flightControllerVersion !== '') {
        FEATURE_CONFIG.features = new Features(CONFIG);
        BEEPER_CONFIG.beepers = new Beepers(CONFIG);
        BEEPER_CONFIG.dshotBeaconConditions = new Beepers(CONFIG, [ "RX_LOST", "RX_SET" ]);

        $('#tabs ul.mode-connected').show();

        //MSP.send_message(MSPCodes.MSP_FEATURE_CONFIG, false, false);
        if (semver.gte(CONFIG.apiVersion, "1.33.0")) {
            //MSP.send_message(MSPCodes.MSP_BATTERY_CONFIG, false, false);
        }
        //MSP.send_message(MSPCodes.MSP_STATUS_EX, false, false);
        //MSP.send_message(MSPCodes.MSP_DATAFLASH_SUMMARY, false, false);

        if (CONFIG.boardType == 0 || CONFIG.boardType == 2) {
            startLiveDataRefreshTimer();
        }
    }

    var sensor_state = $('#sensor-status');
    sensor_state.show();

    var port_picker = $('#portsinput');
    port_picker.hide();

    var dataflash = $('#dataflash_wrapper_global');
    dataflash.show();
}

function onClosed(result) {
    if (result) { // All went as expected
        GUI.log(i18n.getMessage('serialPortClosedOk'));
    } else { // Something went wrong
        GUI.log(i18n.getMessage('serialPortClosedFail'));
    }

    $('#tabs ul.mode-connected').hide();
    $('#tabs ul.mode-connected-cli').hide();
    $('#tabs ul.mode-disconnected').show();

    updateStatusBarVersion();
    updateTopBarVersion();

    var sensor_state = $('#sensor-status');
    sensor_state.hide();

    var port_picker = $('#portsinput');
    port_picker.show();

    var dataflash = $('#dataflash_wrapper_global');
    dataflash.hide();

    var battery = $('#quad-status_wrapper');
    battery.hide();

    //MSP.clearListeners();
    serial.onReceive.removeListener(MAV.read);

    CONFIGURATOR.connectionValid = false;
    CONFIGURATOR.cliValid = false;
    CONFIGURATOR.cliActive = false;
}

function read_serial(info) {
    if (!CONFIGURATOR.cliActive) {
        MSP.read(info);
    } else if (CONFIGURATOR.cliActive) {
        TABS.cli.read(info);
    }
}

function sensor_status(sensors_detected) {
    // initialize variable (if it wasn't)
    if (!sensor_status.previous_sensors_detected) {
        sensor_status.previous_sensors_detected = -1; // Otherwise first iteration will not be run if sensors_detected == 0
    }

    // update UI (if necessary)
    if (sensor_status.previous_sensors_detected == sensors_detected) {
        return;
    }

    // set current value
    sensor_status.previous_sensors_detected = sensors_detected;

    var e_sensor_status = $('div#sensor-status');

    if (have_sensor(sensors_detected, 'acc')) {
        $('.accel', e_sensor_status).addClass('on');
        $('.accicon', e_sensor_status).addClass('active');

    } else {
        $('.accel', e_sensor_status).removeClass('on');
        $('.accicon', e_sensor_status).removeClass('active');
    }

    if (have_sensor(sensors_detected, 'gyro')) { 
        $('.gyro', e_sensor_status).addClass('on');
        $('.gyroicon', e_sensor_status).addClass('active');
    } else {
        $('.gyro', e_sensor_status).removeClass('on');
        $('.gyroicon', e_sensor_status).removeClass('active');
    }

    if (have_sensor(sensors_detected, 'baro')) {
        $('.baro', e_sensor_status).addClass('on');
        $('.baroicon', e_sensor_status).addClass('active');
    } else {
        $('.baro', e_sensor_status).removeClass('on');
        $('.baroicon', e_sensor_status).removeClass('active');
    }

    if (have_sensor(sensors_detected, 'mag')) {
        $('.mag', e_sensor_status).addClass('on');
        $('.magicon', e_sensor_status).addClass('active');
    } else {
        $('.mag', e_sensor_status).removeClass('on');
        $('.magicon', e_sensor_status).removeClass('active');
    }

    if (have_sensor(sensors_detected, 'gps')) {
        $('.gps', e_sensor_status).addClass('on');
    $('.gpsicon', e_sensor_status).addClass('active');
    } else {
        $('.gps', e_sensor_status).removeClass('on');
        $('.gpsicon', e_sensor_status).removeClass('active');
    }

    if (have_sensor(sensors_detected, 'sonar')) {
        $('.sonar', e_sensor_status).addClass('on');
        $('.sonaricon', e_sensor_status).addClass('active');
    } else {
        $('.sonar', e_sensor_status).removeClass('on');
        $('.sonaricon', e_sensor_status).removeClass('active');
    }
}

function have_sensor(sensors_detected, sensor_code) {
    switch(sensor_code) {
        case 'acc':
            return rosflightIO.device.sensors.acc;
        case 'baro':
            return rosflightIO.device.sensors.baro;
        case 'mag':
            return rosflightIO.device.sensors.mag;
        case 'gps':
            return rosflightIO.device.sensors.gps;
        case 'sonar':
            return rosflightIO.device.sensors.sonar;
        case 'gyro': 
            return rosflightIO.device.sensors.gyro;
    }
    return false;
}

function update_dataflash_global() {
    var supportsDataflash = DATAFLASH.totalSize > 0;
    if (supportsDataflash){

         $(".noflash_global").css({
             display: 'none'
         });

         $(".dataflash-contents_global").css({
             display: 'block'
         });

         $(".dataflash-free_global").css({
             width: (100-(DATAFLASH.totalSize - DATAFLASH.usedSize) / DATAFLASH.totalSize * 100) + "%",
             display: 'block'
         });
         $(".dataflash-free_global div").text('Dataflash: free ' + formatFilesize(DATAFLASH.totalSize - DATAFLASH.usedSize));
    } else {
         $(".noflash_global").css({
             display: 'block'
         });

         $(".dataflash-contents_global").css({
             display: 'none'
         });
    }

}

function startLiveDataRefreshTimer() {
    // live data refresh
    GUI.timeout_add('data_refresh', function () { update_live_status(); }, 100);
}

function update_live_status() {

    var statuswrapper = $('#quad-status_wrapper');

    $(".quad-status-contents").css({
       display: 'inline-block'
    });

    if (GUI.active_tab != 'cli') {
        /*MSP.send_message(MSPCodes.MSP_BOXNAMES, false, false);
        if (semver.gte(CONFIG.apiVersion, "1.32.0"))
            MSP.send_message(MSPCodes.MSP_STATUS_EX, false, false);
        else
            MSP.send_message(MSPCodes.MSP_STATUS, false, false);
        MSP.send_message(MSPCodes.MSP_ANALOG, false, false);*/
    }
 
    var active = ((Date.now() - ANALOG.last_received_timestamp) < 300);

    for (var i = 0; i < AUX_CONFIG.length; i++) {
       if (AUX_CONFIG[i] == 'ARM') {
               if (rosflightIO.out_status.armed)
                       $(".armedicon").css({
                               'background-image': 'url(images/icons/cf_icon_armed_active.svg)'
                           });
               else
                       $(".armedicon").css({
                               'background-image': 'url(images/icons/cf_icon_armed_grey.svg)'
                           });
       }
       if (AUX_CONFIG[i] == 'FAILSAFE') {
               if (rosflightIO.out_status.failsafe)
                       $(".failsafeicon").css({
                               'background-image': 'url(images/icons/cf_icon_failsafe_active.svg)'
                           });
               else
                       $(".failsafeicon").css({
                               'background-image': 'url(images/icons/cf_icon_failsafe_grey.svg)'
                           });
       }
    }
    if (ANALOG != undefined) {
    var nbCells = Math.floor(ANALOG.voltage / BATTERY_CONFIG.vbatmaxcellvoltage) + 1;
    if (ANALOG.voltage == 0)
           nbCells = 1;

       var min = BATTERY_CONFIG.vbatmincellvoltage * nbCells;
       var max = BATTERY_CONFIG.vbatmaxcellvoltage * nbCells;
       var warn = BATTERY_CONFIG.vbatwarningcellvoltage * nbCells;

       $(".battery-status").css({
          width: ((ANALOG.voltage - min) / (max - min) * 100) + "%",
          display: 'inline-block'
       });

       if (active) {
           $(".linkicon").css({
               'background-image': 'url(images/icons/cf_icon_link_active.svg)'
           });
       } else {
           $(".linkicon").css({
               'background-image': 'url(images/icons/cf_icon_link_grey.svg)'
           });
       }

       if (ANALOG.voltage < warn) {
           $(".battery-status").css('background-color', '#D42133');
       } else  {
           $(".battery-status").css('background-color', '#59AA29');
       }

       $(".battery-legend").text(ANALOG.voltage + " V");
    }

    statuswrapper.show();
    GUI.timeout_remove('data_refresh');
    startLiveDataRefreshTimer();
}

function specificByte(num, pos) {
    return 0x000000FF & (num >> (8 * pos));
}

function bit_check(num, bit) {
    return ((num >> bit) % 2 != 0);
}

function bit_set(num, bit) {
    return num | 1 << bit;
}

function bit_clear(num, bit) {
    return num & ~(1 << bit);
}

function update_dataflash_global() {
    function formatFilesize(bytes) {
        if (bytes < 1024) {
            return bytes + "B";
        }
        var kilobytes = bytes / 1024;

        if (kilobytes < 1024) {
            return Math.round(kilobytes) + "kB";
        }

        var megabytes = kilobytes / 1024;

        return megabytes.toFixed(1) + "MB";
    }

    var supportsDataflash = DATAFLASH.totalSize > 0;

    if (supportsDataflash){
        $(".noflash_global").css({
           display: 'none'
        });

        $(".dataflash-contents_global").css({
           display: 'block'
        });

        $(".dataflash-free_global").css({
           width: (100-(DATAFLASH.totalSize - DATAFLASH.usedSize) / DATAFLASH.totalSize * 100) + "%",
           display: 'block'
        });
        $(".dataflash-free_global div").text('Dataflash: free ' + formatFilesize(DATAFLASH.totalSize - DATAFLASH.usedSize));
     } else {
        $(".noflash_global").css({
           display: 'block'
        });

        $(".dataflash-contents_global").css({
           display: 'none'
        });
     }
}
