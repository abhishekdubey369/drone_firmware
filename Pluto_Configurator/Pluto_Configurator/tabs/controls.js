/**
 * Created by Neha on 25-10-2016.
 */
'use strict';

var refreshIntervalId = null,
    rc_array_buffer = [],
    requested_properties_live_log = [],
    i,
    h,
    controlsCheck = false,
    joystickCheck = false,
    armVal = 1200,
    joystick_array = [],
    buffering_set_rc = [],
    buffer_delay = false,
    activeArm = false,
    minVal = 1000,
    arm_enable = false;

TABS.controls = {
    yaw_fix: 0.0
};
TABS.controls.initialize = function (callback) {
    var self = this;
    var requested_properties = [],
        samples = 0,
        requests = 0,
        log_buffer = [];

    if (GUI.active_tab != 'controls') {
        GUI.active_tab = 'controls';
        googleAnalytics.sendAppView('Controls');
    }

    function load_status() {
        MSP.send_message(MSP_codes.MSP_STATUS, false, false, load_ident);
    }

    function load_ident() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);
    }

    function load_config() {
        MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, load_misc_data);
    }

    function load_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/controls.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, load_status);

    function process_html() {
        // translate to user-selected language
        console.log("processing html");
        localize();

        // initialize 3D
        self.initialize3D();
        // set roll in interactive block
        $('span.roll').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));
        // set pitch in interactive block
        $('span.pitch').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));
        // set heading in interactive block
        $('span.heading').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));

        self.initializeInstruments();
        // UI Hooks

        $('a.resetSettings').click(function () {
            MSP.send_message(MSP_codes.MSP_RESET_CONF, false, false, function () {
                GUI.log(chrome.i18n.getMessage('initialSetupSettingsRestored'));
                GUI.tab_switch_cleanup(function () {
                    TABS.controls.initialize();
                });
            });
        });
        // display current yaw fix value (important during tab re-initialization)
        $('div#interactive_block > a.reset').text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));
        // reset yaw button hook
        $('div#interactive_block > a.reset').click(function () {
            self.yaw_fix = SENSOR_DATA.kinematics[2] * -1.0;
            $(this).text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));
            console.log('YAW reset to 0 deg, fix: ' + self.yaw_fix + ' deg');
        });

        // cached elements
        var bat_voltage_e = $('.bat-voltage'),
            bat_mah_drawn_e = $('.bat-mah-drawn'),
            bat_mah_drawing_e = $('.bat-mah-drawing'),
            rssi_e = $('.rssi'),
            gpsFix_e = $('.gpsFix'),
            gpsSats_e = $('.gpsSats'),
            gpsLat_e = $('.gpsLat'),
            gpsLon_e = $('.gpsLon'),
            roll_e = $('dd.roll'),
            pitch_e = $('dd.pitch'),
            heading_e = $('dd.heading');

        function get_slow_data() {

            MSP.send_message(MSP_codes.MSP_STATUS);
            MSP.send_message(MSP_codes.MSP_ANALOG, false, false, function () {
                bat_voltage_e.text(chrome.i18n.getMessage('initialSetupBatteryValue', [ANALOG.voltage]));
                bat_mah_drawn_e.text(chrome.i18n.getMessage('initialSetupBatteryMahValue', [ANALOG.mAhdrawn]));
                bat_mah_drawing_e.text(chrome.i18n.getMessage('initialSetupBatteryAValue', [ANALOG.amperage.toFixed(2)]));
                rssi_e.text(chrome.i18n.getMessage('initialSetupRSSIValue', [((ANALOG.rssi / 1023) * 100).toFixed(0)]));
            });
            if (have_sensor(CONFIG.activeSensors, 'gps')) {
                MSP.send_message(MSP_codes.MSP_RAW_GPS, false, false, function () {
                    gpsFix_e.html((GPS_DATA.fix) ? chrome.i18n.getMessage('gpsFixTrue') : chrome.i18n.getMessage('gpsFixFalse'));
                    gpsSats_e.text(GPS_DATA.numSat);
                    gpsLat_e.text((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
                    gpsLon_e.text((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');
                });
            }
        }

        function get_fast_data() {

            MSP.send_message(MSP_codes.MSP_ATTITUDE, false, false, function () {
                roll_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[0]]));
                pitch_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[1]]));
                heading_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[2]]));
                self.render3D();
                self.updateInstruments();
            });
        }


        $('#enableGamepad').prop('checked', false);
        $('#enableGamepad').prop('disabled', false);

        loadGamepadModule();

        $('#GPStatus').hide();

        $('#enableGamepad').change(function () {
            if ($(this).is(':checked')) {


                if (refreshIntervalId != null) {
                    window.clearInterval(refreshIntervalId);
                    console.log("refreshIntervalId", refreshIntervalId);
                }

                refreshIntervalId = window.setInterval(update, 150);

                joystickCheck = true;
                var statusis = document.getElementById('armField').value;

                function update() {

                    var armValue = joystick_array[7];
                    if (armValue == 1) {
                        if (arm_enable)
                            arm_enable = false;
                        else
                            arm_enable = true;
                    }


                    if (arm_enable) {

                        var val = 1500;
                        var statusis = "GAMEPAD ARMED";
                        joystick_array[7] = val;
                    }
                    else {
                        var statusis = "GAMEPAD DISARMED";
                        var val = 1200;
                        joystick_array[7] = val;
                    }

                    if (statusis == "GAMEPAD DISARMED") {
                        armField.innerHTML = "GAMEPAD DISARMED";
                    }
                    else {
                        armField.innerHTML = "GAMEPAD ARMED";
                    }

                    console.log("gamepad values:" + joystick_array);
                    MSP.setRawRx(joystick_array);
                }

                console.log(refreshIntervalId);
                $('#activeGP').css({'background': '#a1f89c'});
                $('#activeControls').css({'background': 'white'});
                $('#GPStatus').show();
                $('#ControlsStatus').hide();
            }
            else {
                console.log("in else");
                window.clearInterval(refreshIntervalId);
                console.log(refreshIntervalId);
                joystick_array[7] = 1200;
                armField.innerHTML = "GAMEPAD DISARMED";

                console.log(joystick_array);
                MSP.setRawRx(joystick_array);

                $('#activeControls').css({'background': '#a1f89c'});
                $('#activeGP').css({'background': 'white'});

                $('#GPStatus').hide();
                $('#ControlsStatus').show();
            }

        });

        if (joystickCheck == false) {

            if (!($('#enableGamepad').is(':checked'))) {
                activeArm = false;

                $('#arm').on('click', function () {
                    var currentvalue = document.getElementById('arm').value;
                    if (currentvalue == "DISARMED") {
                        document.getElementById("arm").value = "ARMED";
                        activeArm = true;
                        $('div.insert input').trigger('input');
                    }
                    else {
                        document.getElementById("arm").value = "DISARMED";
                        activeArm = false;
                        $('div.insert input').trigger('input');
                    }

                });

                $('div.insert input').on('input', function () {

                    for (i = 0; i < 4; i++) {
                        var val = parseInt($('div.insert input').eq(i).val());
                        joystick_array[i] = val;
                    }

                    for (i = 0; i < 2; i++) {
                        var jvalue = ((joystick_array[i] * 25) + 1500);
                        joystick_array[i] = jvalue;
                    }
                    jvalue = ((joystick_array[2] * 10) + 1000);
                    joystick_array[2] = jvalue;

                    jvalue = ((joystick_array[3] * 5) + 1500);
                    joystick_array[3] = jvalue;

                    joystick_array[4] = minVal;
                    joystick_array[5] = minVal;
                    joystick_array[6] = minVal;
                    if (activeArm) {
                        joystick_array[7] = 1500;
                    }
                    else {
                        joystick_array[7] = minVal;
                    }

                    console.log(joystick_array);
                    MSP.setRawRx(joystick_array);

                });
            }
        }

        $('put[type="number"]').each(function () {
            (this).on('keyup', function () {
                if ($(this).val() > Number($(this).attr("max"))) {
                    var val = $(this).val().slice(0, $(this).attr("max").length);
                    $(this).val(val);
                }
            });
        });

        $('a.log_file').click(prepare_file);

        $('a.logging').click(function () {
            if (GUI.connected_to) {
                if (fileEntry != null) {
                    var clicks = $(this).data('clicks');

                    if (!clicks) {
                        // reset some variables before start
                        samples = 0;
                        requests = 0;
                        log_buffer = [];
                        requested_properties = [];

                        $('.properties input:checked').each(function () {
                            requested_properties.push($(this).prop('name'));
                        });

                        if (requested_properties.length) {
                            // print header for the csv file
                            print_head();

                            var log_data_poll = function () {
                                if (requests) {
                                    // save current data (only after everything is initialized)
                                    crunch_data();
                                }

                                // request new
                                for (var i = 0; i < requested_properties.length; i++, requests++) {
                                    MSP.send_message(MSP_codes[requested_properties[i]]);
                                }
                            };

                            GUI.interval_add('log_data_poll', log_data_poll, parseInt($('select.speed').val()), true); // refresh rate goes here
                            GUI.interval_add('write_data', function write_data() {
                                if (log_buffer.length) { // only execute when there is actual data to write
                                    if (fileWriter.readyState == 0 || fileWriter.readyState == 2) {
                                        append_to_file(log_buffer.join('\n'));
                                        $('.samples').text(samples += log_buffer.length);
                                        log_buffer = [];
                                    } else {
                                        console.log('IO having trouble keeping up with the data flow');
                                    }
                                }
                            }, 1000);

                            GUI.interval_add('process_data_poll', function process_data_poll() {
                                var sample = "";
                                for (var i = 0; i < requested_properties.length; i++) {
                                    switch (requested_properties[i]) {
                                        case 'MSP_RAW_IMU':
                                            sample += " Sensor Gyro: ";
                                            sample += SENSOR_DATA.gyroscope;
                                            sample += ',' + " Sensor Acc: ";
                                            sample += SENSOR_DATA.accelerometer;
                                            sample += ',' + " Sensor Mag: ";
                                            sample += SENSOR_DATA.magnetometer;
                                            break;
                                        case 'MSP_ATTITUDE':
                                            sample += " Kinematics : ";
                                            sample += SENSOR_DATA.kinematics[0];
                                            sample += ',' + SENSOR_DATA.kinematics[1];
                                            sample += ',' + SENSOR_DATA.kinematics[2];
                                            break;
                                        case 'MSP_ALTITUDE':
                                            sample += " Altitude: ";
                                            sample += SENSOR_DATA.altitude;
                                            break;
                                        case 'MSP_RAW_GPS':
                                            sample += " Raw GPS: ";
                                            sample += GPS_DATA.fix;
                                            sample += ',' + GPS_DATA.numSat;
                                            sample += ',' + (GPS_DATA.lat / 10000000);
                                            sample += ',' + (GPS_DATA.lon / 10000000);
                                            sample += ',' + GPS_DATA.alt;
                                            sample += ',' + GPS_DATA.speed;
                                            sample += ',' + GPS_DATA.ground_course;
                                            break;
                                        case 'MSP_ANALOG':
                                            sample += " Analog Vol: ";
                                            sample += ANALOG.voltage;
                                            sample += ',' + " Analog Amp: ";
                                            sample += ANALOG.amperage;
                                            sample += ',' + " Analog mAhDrawn: ";
                                            sample += ANALOG.mAhdrawn;
                                            sample += ',' + " Analog RSSI: ";
                                            sample += ANALOG.rssi;
                                            break;
                                        case 'MSP_RC':
                                            for (var chan = 0; chan < RC.active_channels; chan++) {
                                                sample += ' RC' + chan + ': ';
                                                sample += RC.channels[chan] + ',';
                                            }
                                            break;
                                        case 'MSP_MOTOR':
                                            for (var motor = 0; motor < MOTOR_DATA.length; motor++) {
                                                sample += ' Motor' + motor + ': ';
                                                sample += MOTOR_DATA[motor] + ',';
                                            }
                                            break;
                                        case 'MSP_DEBUG':
                                            for (var debug = 0; debug < SENSOR_DATA.debug.length; debug++) {
                                                sample += ' Debug' + debug + ': ';
                                                sample += SENSOR_DATA.debug[debug] + ',';
                                            }
                                            break;
                                    }
                                }
                                GUI.log(sample);
                            }, 1000);

                            $('.speed').prop('disabled', true);
                            $(this).text(chrome.i18n.getMessage('loggingStop'));
                            $(this).data("clicks", !clicks);
                        } else {
                            GUI.log(chrome.i18n.getMessage('loggingErrorOneProperty'));
                        }
                    } else {
                        GUI.interval_kill_all();
                        $('.speed').prop('disabled', false);
                        $(this).text(chrome.i18n.getMessage('loggingStart'));
                        $(this).data("clicks", !clicks);
                    }
                } else {
                    GUI.log(chrome.i18n.getMessage('loggingErrorLogFile'));
                }
            } else {
                GUI.log(chrome.i18n.getMessage('loggingErrorNotConnected'));
            }
        });

        chrome.storage.local.get('logging_file_entry_gamepad', function (result) {
            if (result.logging_file_entry_gamepad) {
                chrome.fileSystem.restoreEntry(result.logging_file_entry_gamepad, function (entry) {
                    fileEntry = entry;
                    prepare_writer(true);
                });
            }
        });

        GUI.interval_add('setup_data_pull_fast', get_fast_data, 33, true); // 30 fps
        GUI.interval_add('setup_data_pull_slow', get_slow_data, 250, true); // 4 fps
        GUI.content_ready(callback);
    }

    function print_head() {
        var head = "timestamp";

        for (var i = 0; i < requested_properties.length; i++) {
            switch (requested_properties[i]) {
                case 'MSP_RAW_IMU':
                    head += ',' + 'gyroscopeX';
                    head += ',' + 'gyroscopeY';
                    head += ',' + 'gyroscopeZ';

                    head += ',' + 'accelerometerX';
                    head += ',' + 'accelerometerY';
                    head += ',' + 'accelerometerZ';

                    head += ',' + 'magnetometerX';
                    head += ',' + 'magnetometerY';
                    head += ',' + 'magnetometerZ';
                    break;
                case 'MSP_ATTITUDE':
                    head += ',' + 'kinematicsX';
                    head += ',' + 'kinematicsY';
                    head += ',' + 'kinematicsZ';
                    break;
                case 'MSP_ALTITUDE':
                    head += ',' + 'altitude';
                    break;
                case 'MSP_RAW_GPS':
                    head += ',' + 'gpsFix';
                    head += ',' + 'gpsNumSat';
                    head += ',' + 'gpsLat';
                    head += ',' + 'gpsLon';
                    head += ',' + 'gpsAlt';
                    head += ',' + 'gpsSpeed';
                    head += ',' + 'gpsGroundCourse';
                    break;
                case 'MSP_ANALOG':
                    head += ',' + 'voltage';
                    head += ',' + 'amperage';
                    head += ',' + 'mAhdrawn';
                    head += ',' + 'rssi';
                    break;
                case 'MSP_RC':
                    for (var chan = 0; chan < RC.active_channels; chan++) {
                        head += ',' + 'RC' + chan;
                    }
                    break;
                case 'MSP_MOTOR':
                    for (var motor = 0; motor < MOTOR_DATA.length; motor++) {
                        head += ',' + 'Motor' + motor;
                    }
                    break;
                case 'MSP_DEBUG':
                    for (var debug = 0; debug < SENSOR_DATA.debug.length; debug++) {
                        head += ',' + 'Debug' + debug;
                    }
                    break;
            }
        }

        append_to_file(head);
    }

    function crunch_data() {
        var d = new Date();
        var date = d.toISOString().substring(0, 10);
        var time = d.toLocaleTimeString();
        time += ":" + d.getMilliseconds();
        var sample = [date, time].join(' @ ');

        for (var i = 0; i < requested_properties.length; i++) {
            switch (requested_properties[i]) {
                case 'MSP_RAW_IMU':
                    sample += ',' + SENSOR_DATA.gyroscope;
                    sample += ',' + SENSOR_DATA.accelerometer;
                    sample += ',' + SENSOR_DATA.magnetometer;
                    break;
                case 'MSP_ATTITUDE':
                    sample += ',' + SENSOR_DATA.kinematics[0];
                    sample += ',' + SENSOR_DATA.kinematics[1];
                    sample += ',' + SENSOR_DATA.kinematics[2];
                    break;
                case 'MSP_ALTITUDE':
                    sample += ',' + SENSOR_DATA.altitude;
                    break;
                case 'MSP_RAW_GPS':
                    sample += ',' + GPS_DATA.fix;
                    sample += ',' + GPS_DATA.numSat;
                    sample += ',' + (GPS_DATA.lat / 10000000);
                    sample += ',' + (GPS_DATA.lon / 10000000);
                    sample += ',' + GPS_DATA.alt;
                    sample += ',' + GPS_DATA.speed;
                    sample += ',' + GPS_DATA.ground_course;
                    break;
                case 'MSP_ANALOG':
                    sample += ',' + ANALOG.voltage;
                    sample += ',' + ANALOG.amperage;
                    sample += ',' + ANALOG.mAhdrawn;
                    sample += ',' + ANALOG.rssi;
                    break;
                case 'MSP_RC':
                    for (var chan = 0; chan < RC.active_channels; chan++) {
                        sample += ',' + RC.channels[chan];
                    }
                    break;
                case 'MSP_MOTOR':
                    sample += ',' + MOTOR_DATA;
                    break;
                case 'MSP_DEBUG':
                    sample += ',' + SENSOR_DATA.debug;
                    break;
            }
        }

        log_buffer.push(sample);
    }

    // IO related methods
    var fileEntry = null,
        fileWriter = null;

    function prepare_file() {
        // create or load the file
        chrome.fileSystem.chooseEntry({
            type: 'saveFile',
            suggestedName: 'cleanflight_data_log',
            accepts: [{extensions: ['csv']}]
        }, function (entry) {
            if (!entry) {
                console.log('No file selected');
                return;
            }

            fileEntry = entry;

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(fileEntry, function (path) {
                console.log('Log file path: ' + path);
            });

            // change file entry from read only to read/write
            chrome.fileSystem.getWritableEntry(fileEntry, function (fileEntryWritable) {
                // check if file is writable
                chrome.fileSystem.isWritableEntry(fileEntryWritable, function (isWritable) {
                    if (isWritable) {
                        fileEntry = fileEntryWritable;

                        // save entry for next use
                        chrome.storage.local.set({'logging_file_entry_gamepad': chrome.fileSystem.retainEntry(fileEntry)});

                        // reset sample counter in UI
                        $('.samples').text(0);

                        prepare_writer();
                    } else {
                        console.log('File appears to be read only, sorry.');
                    }
                });
            });
        });
    }

    function prepare_writer(retaining) {
        fileEntry.createWriter(function (writer) {
            fileWriter = writer;

            fileWriter.onerror = function (e) {
                console.error(e);

                // stop logging if the procedure was/is still running
                if ($('a.logging').data('clicks')) $('a.logging').click();
            };

            fileWriter.onwriteend = function () {
                $('.size').text(bytesToSize(fileWriter.length));
            };

            if (retaining) {
                chrome.fileSystem.getDisplayPath(fileEntry, function (path) {
                    GUI.log(chrome.i18n.getMessage('loggingAutomaticallyRetained', [path]));
                });
            }

            // update log size in UI on fileWriter creation
            $('.size').text(bytesToSize(fileWriter.length));
        }, function (e) {
            // File is not readable or does not exist!
            console.error(e);

            if (retaining) {
                fileEntry = null;
            }
        });
    }

    function append_to_file(data) {
        if (fileWriter.position < fileWriter.length) {
            fileWriter.seek(fileWriter.length);
        }

        fileWriter.write(new Blob([data + '\n'], {type: 'text/plain'}));
    }
};

TABS.controls.initializeInstruments = function () {
    var options = {size: 90, showBox: false, img_directory: 'images/flightindicators/'};
    var attitude = $.flightIndicator('#attitude', 'attitude', options);
    var heading = $.flightIndicator('#heading', 'heading', options);
    this.updateInstruments = function () {
        attitude.setRoll(SENSOR_DATA.kinematics[0]);
        attitude.setPitch(SENSOR_DATA.kinematics[1]);
        heading.setHeading(SENSOR_DATA.kinematics[2]);
    };
};
TABS.controls.initialize3D = function (compatibility) {

    var self = this,
        loader, canvas, wrapper, renderer, camera, scene, light, light2, modelWrapper, model, model_file,
        useWebGlRenderer = false;
    canvas = $('.model-and-info #canvas');
    wrapper = $('.model-and-info #canvas_wrapper');
    // webgl capability detector
    // it would seem the webgl "enabling" through advanced settings will be ignored in the future
    // and webgl will be supported if gpu supports it by default (canary 40.0.2175.0), keep an eye on this one
    var detector_canvas = document.createElement('canvas');
    if (window.WebGLRenderingContext && (detector_canvas.getContext('webgl') || detector_canvas.getContext('experimental-webgl'))) {
        renderer = new THREE.WebGLRenderer({canvas: canvas.get(0), alpha: true, antialias: true});
        useWebGlRenderer = true;
    } else {
        renderer = new THREE.CanvasRenderer({canvas: canvas.get(0), alpha: true});
    }
    // initialize render size for current canvas size
    renderer.setSize(wrapper.width() * 2, wrapper.height() * 2);
//    // modelWrapper adds an extra axis of rotation to avoid gimbal lock with the euler angles
    modelWrapper = new THREE.Object3D();
//
    // load the model including materials
    if (useWebGlRenderer) {
        model_file = mixerList[CONFIG.multiType - 1].model;
    } else {
        model_file = 'fallback'
    }
    // Temporary workaround for 'custom' model until akfreak's custom model is merged.
    var useLegacyCustomModel = false;
    if (model_file == 'custom') {
        //  model_file = 'fallback';
        model_file = 'quad_x';
        useLegacyCustomModel = true;
    }
    // setup scene
    scene = new THREE.Scene();
    loader = new THREE.JSONLoader();
    loader.load('./resources/models/' + model_file + '.json', function (geometry, materials) {
        var modelMaterial = new THREE.MeshFaceMaterial(materials);
        model = new THREE.Mesh(geometry, modelMaterial);
        model.scale.set(15, 15, 15);
        modelWrapper.add(model);
        scene.add(modelWrapper);
    });
    // stationary camera
    camera = new THREE.PerspectiveCamera(50, wrapper.width() / wrapper.height(), 1, 10000);
    // some light
    light = new THREE.AmbientLight(0x404040);
    light2 = new THREE.DirectionalLight(new THREE.Color(1, 1, 1), 1.5);
    light2.position.set(0, 1, 0);
    // move camera away from the model
    camera.position.z = 125;
    // add camera, model, light to the foreground scene
    scene.add(light);
    scene.add(light2);
    scene.add(camera);
    scene.add(modelWrapper);
    this.render3D = function () {

        if (!model) {
            return;
        }
        // compute the changes
        model.rotation.x = (SENSOR_DATA.kinematics[1] * -1.0) * 0.017453292519943295;
        modelWrapper.rotation.y = ((SENSOR_DATA.kinematics[2] * -1.0) - self.yaw_fix) * 0.017453292519943295;
        model.rotation.z = (SENSOR_DATA.kinematics[0] * -1.0) * 0.017453292519943295;
        // draw
        renderer.render(scene, camera);
    };
    // handle canvas resize
    this.resize3D = function () {
        renderer.setSize(wrapper.width() * 2, wrapper.height() * 2);
        camera.aspect = wrapper.width() / wrapper.height();
        camera.updateProjectionMatrix();
        self.render3D();
    };
    $(window).on('resize', this.resize3D);
};

TABS.controls.cleanup = function (callback) {
    removeGamepadModule();
    joystick_array[7] = 1200;
    MSP.setRawRx(joystick_array);

    if (refreshIntervalId != null) {
        window.clearInterval(refreshIntervalId);
    }
    console.log("refreshIntervalId afterwards", refreshIntervalId);
    GUI.interval_kill_all();

    $(window).off('resize', this.resize3D);

    if (callback) callback();
};
