//% color="#00C04A" weight=100 icon="\uf1b9" block="智能IIC实战巡线"
namespace AnalogLineFollow {
    // ==========================================
    // 全局变量与状态记忆 
    // ==========================================
    let _kp = 0.07; let _ki = 0; let _kd = 0.09;
    let _prevError = 0; let _integral = 0;
    let _baseSpeed = 60; let _brake = 1;
    let _integralLimit = 1500; 
    let _lastLeftSpeed = 0; let _lastRightSpeed = 0;
    let _isWhiteLine = false; 
    let _isFirstRun = true;    

    let _leftMotorScale = 1.0;
    let _rightMotorScale = 1.0;

    // 四路灰度（统一读取后的稳定状态：true=踩线）
    let _l2 = false; let _l1 = false; let _r1 = false; let _r2 = false;

    // 超时保护与抗积分饱和阈值
    const _INTEGRAL_DEADBAND = 400;
    const _TURN_TIMEOUT = 2500;
    const _CROSS_WAIT_TIMEOUT = 3000;
    const _CROSS_OVERALL_TIMEOUT = 15000;

    export enum TurnDir {
        //% block="左"
        Left,
        //% block="右"
        Right
    }

    export enum LineType {
        //% block="黑线(白底)"
        Black,
        //% block="白线(黑底)"
        White
    }

    export enum IntersectType {
        //% block="左路口"
        Left,
        //% block="右路口"
        Right,
        //% block="十字/停止线"
        Cross,
        //% block="任意路口"
        Any
    }

    export enum IntersectAction {
        //% block="精准急刹"
        Stop,
        //% block="平滑刹车"
        SmoothBrake,
        //% block="冲过路口(盲开)"
        CrossOver
    }

    export enum SearchStrategy {
        //% block="仅前后探测"
        FrontBack,
        //% block="仅左右探测"
        LeftRight,
        //% block="十字全探测(前后左右)"
        CrossAll
    }

    function _setMotorSpeed(left: number, right: number): void {
        let finalL = Math.max(-100, Math.min(100, left * _leftMotorScale));
        let finalR = Math.max(-100, Math.min(100, right * _rightMotorScale));
        neZha.setMotorSpeed(neZha.MotorList.M1, Math.round(finalL));
        neZha.setMotorSpeed(neZha.MotorList.M2, Math.round(finalR));
    }

    // 重置 PID 记忆（积分/微分），避免急刹后或重新起步时出现微分尖峰
    function _resetPIDState(): void {
        _integral = 0;
        _prevError = 0;
        _isFirstRun = true;
    }

    // 统一读取四路灰度并做黑白反转，供所有判路逻辑复用
    function _refreshChannels(): void {
        PlanetX_Basic.Trackbit_get_state_value();
        let raw_l2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.One, PlanetX_Basic.TrackbitType.State_1);
        let raw_l1 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Two, PlanetX_Basic.TrackbitType.State_1);
        let raw_r1 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Three, PlanetX_Basic.TrackbitType.State_1);
        let raw_r2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Four, PlanetX_Basic.TrackbitType.State_1);
        _l2 = _isWhiteLine ? raw_l2 : !raw_l2;
        _l1 = _isWhiteLine ? raw_l1 : !raw_l1;
        _r1 = _isWhiteLine ? raw_r1 : !raw_r1;
        _r2 = _isWhiteLine ? raw_r2 : !raw_r2;
    }

    // =================【第一梯队：初始化与校准】=================

    //% block="初始化 IIC巡线 Kp $p Ki $i Kd $d 基础速度 $baseSpeed 刹车 $brake 赛道 $line"
    //% p.defl=0.07 i.defl=0 d.defl=0.09 baseSpeed.defl=60 brake.defl=1
    //% weight=100
    export function setPID(p: number, i: number, d: number, baseSpeed: number, brake: number, line: LineType): void {
        _kp = p; _ki = i; _kd = d; _baseSpeed = baseSpeed; _brake = brake;
        _isWhiteLine = (line === LineType.White);
        _resetPIDState();
    }

    //% block="校准底盘：左轮动力 $left | 右轮动力 $right"
    //% left.defl=100 left.min=50 left.max=100
    //% right.defl=100 right.min=50 right.max=100
    //% weight=99
    export function calibrateMotor(left: number, right: number): void {
        _leftMotorScale = left / 100.0; _rightMotorScale = right / 100.0;
    }

    // =================【第二梯队：核心巡线】=================

    //% block="执行一次PID灰度巡线"
    //% weight=95
    export function pidRun(): void {
        let error = PlanetX_Basic.TrackBit_get_offset();
        
        // 🚀 终极护盾其一：【辅路抗干扰过滤器】(无视单侧辅路)
        _refreshChannels();

        if (_l1 || _r1) { 
            // 只要主体在主线上，且遇到单侧辅路，直接锁死 error 强制直行！
            if (_r2 && !_l2) { error = 0; _prevError = 0; } 
            else if (_l2 && !_r2) { error = 0; _prevError = 0; }
        }

        if (_isFirstRun) { _prevError = error; _isFirstRun = false; }

        // 抗积分饱和：只在接近中线时累计积分，脱轨时不累计
        if (Math.abs(error) <= _INTEGRAL_DEADBAND) {
            _integral += error;
            _integral = Math.max(-_integralLimit, Math.min(_integralLimit, _integral));
        }

        let derivative = error - _prevError;
        let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);
        _prevError = error;

        let curveSharpness = Math.abs(error) / 100;
        let dynamicBaseSpeed = Math.max(15, _baseSpeed - (curveSharpness * _brake));

        let leftSpeed = dynamicBaseSpeed + adjustment;
        let rightSpeed = dynamicBaseSpeed - adjustment;

        _lastLeftSpeed = leftSpeed; _lastRightSpeed = rightSpeed;
        _setMotorSpeed(leftSpeed, rightSpeed);
    }

    // =================【第三梯队：复杂赛道处理】=================

    //% block="PID巡线 经过 $count 个 $intersectType 后 $action | 冲过速度 $crossSpeed 持续(ms) $crossTime"
    //% count.defl=1 crossSpeed.defl=40 crossTime.defl=300
    //% weight=88
    export function pidCrossMultiple(count: number, intersectType: IntersectType, action: IntersectAction, crossSpeed: number, crossTime: number): void {
        let metCount = 0; 
        let metStreak = 0;
        let overallTimeout = input.runningTime() + _CROSS_OVERALL_TIMEOUT;

        while (metCount < count && input.runningTime() < overallTimeout) {
            _refreshChannels();

            let isMet = false;
            if (intersectType === IntersectType.Left) isMet = _l2;
            else if (intersectType === IntersectType.Right) isMet = _r2;
            else if (intersectType === IntersectType.Cross) isMet = (_l2 && _r2);
            else if (intersectType === IntersectType.Any) isMet = (_l2 || _r2);

            if (isMet) {
                // 去抖：连续两次采样都命中才计一个路口
                metStreak++;
                if (metStreak < 2) { basic.pause(3); continue; }
                metStreak = 0;
                metCount++; 

                if (metCount >= count) {
                    if (action === IntersectAction.Stop) {
                        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(50); 
                    } else if (action === IntersectAction.SmoothBrake) {
                        smoothBrake(10); 
                    } else if (action === IntersectAction.CrossOver) {
                        _setMotorSpeed(crossSpeed, crossSpeed); basic.pause(crossTime);
                        _lastLeftSpeed = crossSpeed; _lastRightSpeed = crossSpeed;
                    }
                    break; 
                } else {
                    // 通过当前路口：继续 PID，直到该路口条件消失；加超时防卡死
                    let waitTimeout = input.runningTime() + _CROSS_WAIT_TIMEOUT;
                    let stillMet = true;
                    while (stillMet && input.runningTime() < waitTimeout) {
                        pidRun(); 
                        stillMet = false;
                        if (intersectType === IntersectType.Left) stillMet = _l2;
                        else if (intersectType === IntersectType.Right) stillMet = _r2;
                        else if (intersectType === IntersectType.Cross) stillMet = (_l2 && _r2);
                        else if (intersectType === IntersectType.Any) stillMet = (_l2 || _r2);
                        if (stillMet) basic.pause(5);
                    }
                    // 若超时仍卡在路口，安全停车并退出
                    if (stillMet) {
                        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
                        return;
                    }
                }
            } else { 
                metStreak = 0;
                // 🚀 终极护盾其二：【算法3.0 防扭曲+防脱轨】
                // 当寻找十字路口且单侧踩线，且中间至少有一个在线上时（确认未脱轨），开启直行锁死！
                if (intersectType === IntersectType.Cross && (_l2 || _r2) && (_l1 || _r1)) {
                    let lockSpeed = Math.max(20, _baseSpeed * 0.5);
                    _setMotorSpeed(lockSpeed, lockSpeed);
                } else {
                    pidRun(); 
                }
                basic.pause(5); 
            }
        }

        // 整体超时仍未走够路口数：安全停车
        if (metCount < count) {
            _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
        }
    }

    //% block="PID定时巡线(无视路口/断线) 基础速度 $baseSpeed 持续(ms) $timeMs"
    //% baseSpeed.defl=45 timeMs.defl=2000
    //% weight=85
    export function pidDashedLine(baseSpeed: number, timeMs: number): void {
        let endTime = input.runningTime() + timeMs;
        let lastLeft = baseSpeed; let lastRight = baseSpeed;
        let wasLost = true; 

        while (input.runningTime() < endTime) {
            _refreshChannels();

            if (_l1 || _r1 || _l2 || _r2) {
                let error = PlanetX_Basic.TrackBit_get_offset();

                if (wasLost) { _prevError = error; _integral = 0; wasLost = false; }

                if (Math.abs(error) <= _INTEGRAL_DEADBAND) {
                    _integral += error; _integral = Math.max(-_integralLimit, Math.min(_integralLimit, _integral));
                }
                let derivative = error - _prevError;
                let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);
                _prevError = error;

                let leftS = baseSpeed + adjustment;
                let rightS = baseSpeed - adjustment;
                lastLeft = leftS; lastRight = rightS;
                _setMotorSpeed(leftS, rightS);
            } else { 
                wasLost = true; 
                _setMotorSpeed(lastLeft, lastRight); 
            }
            basic.pause(10); 
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
        _resetPIDState();
    }

    // =================【第四梯队：智能交互与雷达】=================

    //% block="智能找卡并播报 | 探测模式 $strategy 播报次数 $count 间隔(ms) $interval | 搜索超时(ms) $timeout 搜索速度 $speed 探测步距(ms) $stepTime"
    //% count.defl=2 interval.defl=2000 timeout.defl=10000 speed.defl=40 stepTime.defl=300
    //% weight=75
    export function searchAndReadRFID(strategy: SearchStrategy, count: number, interval: number, timeout: number, speed: number, stepTime: number): void {
        let startTime = input.runningTime();
        let cardFound = false;
        let cardData = "";
        let positionState = 0; 

        function tempMove(lSpeed: number, rSpeed: number, timeMs: number) {
            _setMotorSpeed(lSpeed, rSpeed); basic.pause(timeMs);
            _setMotorSpeed(0, 0); basic.pause(100); 
        }

        while (input.runningTime() - startTime < timeout) {
            if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }

            if (strategy === SearchStrategy.FrontBack || strategy === SearchStrategy.CrossAll) {
                tempMove(speed, speed, stepTime); positionState = 1;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(-speed, -speed, stepTime); positionState = 0; 
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                
                tempMove(-speed, -speed, stepTime); positionState = 2;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(speed, speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
            }

            if (strategy === SearchStrategy.LeftRight || strategy === SearchStrategy.CrossAll) {
                tempMove(-speed, speed, stepTime); positionState = 3;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(speed, -speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }

                tempMove(speed, -speed, stepTime); positionState = 4;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(-speed, speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
            }
        }

        if (cardFound) {
            control.inBackground(function () {
                for (let i = 0; i < count; i++) {
                    basic.showString(cardData);
                    if (i < count - 1) basic.pause(interval);
                }
                basic.clearScreen();
            });
        } else {
            control.inBackground(function () {
                basic.showIcon(IconNames.No);
                basic.pause(1000);
                basic.clearScreen();
            });
        }

        if (positionState === 1) tempMove(-speed, -speed, stepTime);
        else if (positionState === 2) tempMove(speed, speed, stepTime);
        else if (positionState === 3) tempMove(speed, -speed, stepTime);
        else if (positionState === 4) tempMove(-speed, speed, stepTime);
        
        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0; _lastRightSpeed = 0;
        _resetPIDState(); 
    }

    // =================【第五梯队：姿态对齐】=================

    //% block="自动对齐停止线(十字/T型) | 调整速度 $speed"
    //% speed.defl=30
    //% weight=65
    export function alignToLine(speed: number): void {
        let alignedCount = 0;
        let timeout = input.runningTime() + 3000;

        while (alignedCount < 3 && input.runningTime() < timeout) {
            _refreshChannels();

            let leftSpeed = 0; let rightSpeed = 0;
            if (!_l2) leftSpeed = speed;
            if (!_r2) rightSpeed = speed;

            if (_l2 && _r2) { alignedCount++; leftSpeed = 0; rightSpeed = 0; } 
            else { alignedCount = 0; }

            _setMotorSpeed(leftSpeed, rightSpeed);
            basic.pause(15);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(100);
    }

    //% block="原地死转向 $dir 直到正对线上 | 速度 $speed"
    //% speed.defl=40
    //% weight=60
    export function turnUntilLine(dir: TurnDir, speed: number): void {
        let leftS = dir === TurnDir.Left ? -speed : speed;
        let rightS = dir === TurnDir.Left ? speed : -speed;
        _setMotorSpeed(leftS, rightS); basic.pause(200); 

        let stable = 0;
        let endTime = input.runningTime() + _TURN_TIMEOUT;
        while (input.runningTime() < endTime) {
            let offset = PlanetX_Basic.TrackBit_get_offset();
            if (Math.abs(offset) < 400) {
                stable++;
                if (stable >= 2) break;
            } else {
                stable = 0;
            }
            basic.pause(5);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(50);
    }

    // =================【第六梯队：基础运动】=================

    //% block="以 $speed 速度前进 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=55
    export function forwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); 
        _setMotorSpeed(safeSpeed, safeSpeed);
        _lastLeftSpeed = safeSpeed; _lastRightSpeed = safeSpeed;
        basic.pause(timeMs); 
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
        _resetPIDState();
    }

    //% block="以 $speed 速度后退 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=54
    export function backwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); 
        _setMotorSpeed(-safeSpeed, -safeSpeed);
        _lastLeftSpeed = -safeSpeed; _lastRightSpeed = -safeSpeed;
        basic.pause(timeMs);
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
        _resetPIDState();
    }

    //% block="平滑起步/变速 目标速度 $targetSpeed 步进延迟(ms) $delayMs"
    //% targetSpeed.defl=60 delayMs.defl=20
    //% weight=52
    export function smoothStart(targetSpeed: number, delayMs: number): void {
        let currentS = Math.round((_lastLeftSpeed + _lastRightSpeed) / 2);
        let step = (targetSpeed >= currentS) ? 5 : -5;
        for (let s = currentS; (step > 0 ? s <= targetSpeed : s >= targetSpeed); s += step) {
            _setMotorSpeed(s, s); _lastLeftSpeed = s; _lastRightSpeed = s; basic.pause(delayMs);
        }
        _setMotorSpeed(targetSpeed, targetSpeed); _lastLeftSpeed = targetSpeed; _lastRightSpeed = targetSpeed;
    }

    //% block="平滑刹车 步进延迟(ms) $delayMs"
    //% delayMs.defl=20
    //% weight=50
    export function smoothBrake(delayMs: number): void {
        let steps = 10;
        let leftStep = _lastLeftSpeed / steps; let rightStep = _lastRightSpeed / steps;
        for (let i = 0; i < steps; i++) {
            _lastLeftSpeed -= leftStep; _lastRightSpeed -= rightStep;
            _setMotorSpeed(_lastLeftSpeed, _lastRightSpeed); basic.pause(delayMs);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
        _resetPIDState();
    }

    //% block="停止所有电机"
    //% weight=45
    export function stopMotors(): void {
        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        _resetPIDState();
    }
}