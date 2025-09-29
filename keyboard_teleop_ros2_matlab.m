function keyboard_teleop_ros2_matlab()
    % Inicializa nó ROS 2
    tic
    node = ros2node("/keyboard_teleop");

    % Cria publicador para /robot/cmd_vel
    velPub = ros2publisher(node, "/robot/cmd_vel", "geometry_msgs/Twist");
    cmdMsg = ros2message(velPub);

    % Cria subcribers
    encoderSub = ros2subscriber(node, "/robot/encoder", "std_msgs/Float32MultiArray", @encoderCallback);
    lidarSub = ros2subscriber(node, "/robot/scan", "sensor_msgs/LaserScan", @scanCallback);
    imuSub = ros2subscriber(node, "/robot/imu", "sensor_msgs/Imu", @imuCallback);
    
    % Cria variáveis
    encoder_data = [];
    scanAngles = [];
    scanDistances = [];
    imuLinearAcceleration = [];
    imuAngularVelocity = [];

    % Estados das teclas
    keys.forward = false;
    keys.backward = false;
    keys.left = false;
    keys.right = false;

    % Cria janela para capturar eventos de teclado
    fig = figure('Name', 'Keyboard Teleop', ...
                 'KeyPressFcn', @keyDown, ...
                 'KeyReleaseFcn', @keyUp);

    % Timer para publicar a 30 Hz (0.03333 s)
    timerPeriod = 0.03333;
    t = timer('ExecutionMode', 'fixedRate', ...
              'Period', timerPeriod, ...
              'TimerFcn', @(~,~) timer_callback());

    start(t);

    % --- Funções internas ---
    function keyDown(~, event)
        switch event.Key
            case {'uparrow','w'}
                keys.forward = true;
            case {'downarrow','s'}
                keys.backward = true;
            case {'leftarrow','a'}
                keys.left = true;
            case {'rightarrow','d'}
                keys.right = true;
        end
    end

    function keyUp(~, event)
        switch event.Key
            case {'uparrow','w'}
                keys.forward = false;
            case {'downarrow','s'}
                keys.backward = false;
            case {'leftarrow','a'}
                keys.left = false;
            case {'rightarrow','d'}
                keys.right = false;
        end
    end
    function encoderCallback(msg)
        encoder_data = msg.data;
    end
    function scanCallback(msg)
        distances = msg.ranges;
        valid = find(distances > 0);
        scanAngles = linspace(msg.angle_min, msg.angle_max, length(distances));
        %scanAngles = scanAngles(valid);
        scanDistances = distances;%distances(valid);
    end
    function imuCallback(msg)
        imuLinearAcceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z];
        imuAngularVelocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z];
    end
    function timer_callback()
        % Ganhos máximos como no código original
        Vmax = 25.2 * (0.03);
        Wmax = 25.2 * (0.03/0.173);

        % Calcula velocidades
        linear = 0;
        angular = 0;

        if keys.forward,  linear =  Vmax; end
        if keys.backward, linear = -Vmax; end
        if keys.left,     angular =  Wmax; end
        if keys.right,    angular = -Wmax; end


        % Prepara e envia mensagem
        cmdMsg.linear.x = linear;
        cmdMsg.angular.z = angular;
        send(velPub, cmdMsg);
        disp(cmdMsg.linear.x)
        disp(cmdMsg.angular.z)
        disp(length(imuAngularVelocity))
        disp(length(imuLinearAcceleration))
        disp(length(scanDistances))
        disp(length(scanAngles))
        disp(length(encoder_data))
        toc

    end

    % --- Finalização limpa ---
    cleanup = onCleanup(@() stopTeleop());

    function stopTeleop()
        stop(t);
        delete(t);
        if isvalid(fig), close(fig); end
    end
end