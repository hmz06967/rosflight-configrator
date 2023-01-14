'use strict';

TABS.servos = {};
TABS.servo_fch = ["RC_X_CHN","RC_Y_CHN","RC_Z_CHN","RC_F_CHN", "ARM_CHANNEL", "RC_ATT_OVRD_CHN", "RC_THR_OVRD_CHN", "RC_ATT_CTRL_CHN"];

TABS.servos.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'servos') {
        GUI.active_tab = 'servos';
    }

    function get_servo_configurations() {
        //MSP.send_message(MSPCodes.MSP_SERVO_CONFIGURATIONS, false, false, get_servo_mix_rules);
        get_servo_mix_rules();
    }

    function get_servo_mix_rules() {
        //MSP.send_message(MSPCodes.MSP_SERVO_MIX_RULES, false, false, get_rc_data);
        get_rc_data();
    }

    function get_rc_data() {
            SERVO_CONFIG = [];

        var len  = rosflightIO.out_msg.servo.values.length;
        
            for (let i = 0; i < len; i++) {
                const arr = {
                    'min':                      0,
                    'max':                      2000,
                    'middle':                   1000,
                    'rate':                     0,
                    'indexOfChannelToForward':  0,
                    'reversedInputSources':     0,
                };
                if(TABS.servo_fch[i]!=undefined){
                    var ch = rosflightIO.handle_get_param(TABS.servo_fch[i]);
                    /*if(ch>=0){
                        arr.indexOfChannelToForward =  ch + (i>3 ? 3 : 0);
                    }else{
                        arr.indexOfChannelToForward = -1;
                    }*/
                    arr.indexOfChannelToForward = ch;
                    if(i<4)
                        arr.rate = Math.floor(TABS.map(rosflightIO.handle_get_param("RC_OVRD_DEV"), 0.0, 1.0, -100, 100));
                    else{
                        arr.rate = Math.floor(TABS.map(rosflightIO.handle_get_param("SWITCH_"+(i+1)+"_DIR"), -1, 1, -100, 100));
                    }
                }
                    
                SERVO_CONFIG.push(arr);
            }
        
        //MSP.send_message(MSPCodes.MSP_RC, false, false, get_boxnames_data);
        get_boxnames_data();
    }

    function get_boxnames_data() {
        //MSP.send_message(MSPCodes.MSP_BOXNAMES, false, false, load_html);
        load_html();
    }

    function load_html() {
        $('#content').load("./tabs/servos.html", process_html);
    }
    get_servo_configurations();
    
    function update_ui() {
            
        if (/*semver.lt(CONFIG.apiVersion, "1.12.0") || */ SERVO_CONFIG.length == 0) {
            
            $(".tab-servos").removeClass("supported");
            return;
        }
        
        $(".tab-servos").addClass("supported");
            
        var servoCheckbox = '';
        var servoHeader = '';
        for (var i = 0; i < RC.active_channels-4; i++) {
            servoHeader = servoHeader + '\
                <th >A' + (i+1) + '</th>\
            ';
        }
        servoHeader = servoHeader + '<th style="width: 110px" i18n="servosDirectionAndRate"></th>';

        for (var i = 0; i < RC.active_channels; i++) {
            servoCheckbox = servoCheckbox + '\
                <td class="channel"><input type="checkbox"/></td>\
            ';
        }

        
        $('div.tab-servos table.fields tr.main').append(servoHeader);       
        
        function process_servos(name, alternate, obj) {
        
            $('div.supported_wrapper').show();
            
            $('div.tab-servos table.fields').append('\
                <tr> \
                    <td style="text-align: center">' + name + '</td>\
                    <td class="middle"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].middle + '" /></td>\
                    <td class="min"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].min +'" /></td>\
                    <td class="max"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].max +'" /></td>\
                    ' + servoCheckbox + '\
                    <td class="direction">\
                    </td>\
                </tr> \
            ');

            if (SERVO_CONFIG[obj].indexOfChannelToForward >= 0) {
                $('div.tab-servos table.fields tr:last td.channel input').eq(SERVO_CONFIG[obj].indexOfChannelToForward).prop('checked', true);
            }

            // adding select box and generating options
            $('div.tab-servos table.fields tr:last td.direction').append('\
                <select class="rate" name="rate" index="'+obj+'"></select>\
            ');

            var select = $('div.tab-servos table.fields tr:last td.direction select');


            
            for (var i = 100; i > -101; i--) {
                select.append('<option value="' + i + '">Rate: ' + i + '%</option>');
            }

            // select current rate
            select.val(SERVO_CONFIG[obj].rate);

            $('div.tab-servos table.fields tr:last').data('info', {'obj': obj});
            
            // UI hooks
            
            // only one checkbox for indicating a channel to forward can be selected at a time, perhaps a radio group would be best here.
            $('div.tab-servos table.fields tr:last td.channel input').click(function () {
                if($(this).is(':checked')) {
                    $(this).parent().parent().find('.channel input').not($(this)).prop('checked', false);
                }
            });
        }

        function servos_update(save_configuration_to_eeprom) {
            $('div.tab-servos table.fields tr:not(".main")').each(function () {
                var info = $(this).data('info');


                var selection = $('.channel input', this);
                var channelIndex = parseInt(selection.index(selection.filter(':checked')));
                if (channelIndex == -1) {
                    channelIndex = undefined;
                }
                
                SERVO_CONFIG[info.obj].indexOfChannelToForward = channelIndex;

                
                SERVO_CONFIG[info.obj].middle = parseInt($('.middle input', this).val());
                SERVO_CONFIG[info.obj].min = parseInt($('.min input', this).val());
                SERVO_CONFIG[info.obj].max = parseInt($('.max input', this).val());

                var val = parseInt($('.direction select', this).val());
                SERVO_CONFIG[info.obj].rate = val;
            });
            
            //
            // send data to FC
            //
            //mspHelper.sendServoConfigurations(send_servo_mixer_rules);

            function set_direction(){
               var id = "", 
                   val2 = 0,
                   type = 9;
               if(cb_array.id<4){
                   id = "RC_OVRD_DEV";
                   val2 = TABS.map(SERVO_CONFIG[cb_array.id].rate, -100.0, 100.0, 0.0, 1.0);
               }else{
                   id = "SWITCH_"+(cb_array.id+1)+"_DIR";
                   val2 = SERVO_CONFIG[cb_array.id].rate > 0 ? 1 : -1;
                   type = 6;
               }
               console.log(id, val2);
                MAV.send_message(mavlink10.MAVLINK_MSG_ID_PARAM_SET, {param_id: id, param_value: val2, param_type: type}, ()=>{
                    cb_array.id++;
                    cb_array.cb();  
                }, 0, 1);   
            }
            
            var cb_array = {
                id: 0,
                cb: ()=>{
                   
                   var val = SERVO_CONFIG[cb_array.id].indexOfChannelToForward;
                   //val = (val > 3 ? (val - 3) : val);
                    //console.log(cb_array.id);
                   if(cb_array.id == TABS.servo_fch.length){
                       save_to_eeprom();
                       return;
                   }else{
                       MAV.send_message(mavlink10.MAVLINK_MSG_ID_PARAM_SET, {param_id: TABS.servo_fch[cb_array.id], param_value: val, param_type: 6}, ()=>{
                        set_direction();
                       }, 0, 1);     
                   }
                },         
            };
            
            MAV.send_message(mavlink10.MAVLINK_MSG_ID_PARAM_SET, {param_id: TABS.servo_fch[cb_array.id], param_value: SERVO_CONFIG[cb_array.id].indexOfChannelToForward, param_type: 6}, ()=>{
                set_direction();
            }, 0, 1);
            
            function save_to_eeprom() {
                
                    //MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, function () {
                     rosflightIO.save_parameter(()=>{
                         GUI.log(i18n.getMessage('servosEepromSave'));
                     }); 
                        
                    //});
               
            }

        }

        // drop previous table
        $('div.tab-servos table.fields tr:not(:first)').remove();

        for (var servoIndex = 0; servoIndex < 8; servoIndex++) {
            process_servos('Servo ' + servoIndex, '', servoIndex, false);
        }
        
        
        // UI hooks for dynamically generated elements
        $('table.directions select, table.directions input, table.fields select, table.fields input').change(function () {
            if ($('div.live input').is(':checked')) {
                // apply small delay as there seems to be some funky update business going wrong

                //GUI.timeout_add('servos_update', servos_update, 10);
            }

            else if($(this).hasClass("rate")){
                var val = $(this).val();
                var index = $(this).attr("index");
                var ifc = index < 4 ? 4 : 0;
                while(ifc){
                    $(".direction select[index='"+(ifc-1)+"']").val(val).css("color","#6e0cd9");
                    ifc--;
                }         
            }

        });
        
        $('a.update').click(function () {
            servos_update(true);
        });
        
    }

    function process_html() {
    
        update_ui();

        // translate to user-selected language
        i18n.localizePage();
        
        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function () {
            //MSP.send_message(MSPCodes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.servos.cleanup = function (callback) {
    if (callback) callback();
};
