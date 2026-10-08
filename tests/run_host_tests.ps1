param([string]$Gcc = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testOutput = Join-Path $PSScriptRoot '.build'
$compiler = (Get-Command $Gcc -ErrorAction Stop).Source
$previousPath = $env:PATH
# MinGW的编译器子进程及测试程序同样需要找到工具链DLL。
$env:PATH = (Split-Path -Parent $compiler) + [IO.Path]::PathSeparator + $previousPath
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
Push-Location $projectRoot
try {
    $common = @('-Itests/stubs', '-Iapplication', '-Iapplication/chassis')
    $fc = @('-Itests/chassis_stubs', '-Iapplication', '-Iapplication/command', '-Iapplication/chassis', '-Ibsp/communication', '-Imodules/drive', '-Imodules/input')
    $cases = @(
        @{ Name='test_dual_steer'; Includes=$common },
        @{ Name='test_left_calibration'; Includes=$common },
        @{ Name='test_closed_steer'; Includes=$common },
        @{ Name='test_fc_command'; Includes=$fc },
        @{ Name='test_fc_command_calibration'; Includes=$fc },
        @{ Name='test_fc_pwm_command_integration'; Includes=@('-Itests/pwm_stubs', '-Iapplication', '-Iapplication/command', '-Iapplication/chassis', '-Ibsp/dispatch', '-Imodules/input') },
        @{ Name='test_debug_fc'; Includes=@('-Itests/debug_stubs', '-Iapplication', '-Iapplication/chassis', '-Iapplication/command', '-Imodules/input', '-Ibsp/communication', '-Ibsp/dispatch') },
        @{ Name='test_vehicle_status'; Includes=@('-Iapplication') },
        @{ Name='test_vehicle_status_config'; Includes=@('-Iapplication') },
        @{ Name='test_lighting_status'; Includes=@('-Itests/lighting_stubs', '-Iapplication', '-Imodules/lighting') },
        @{ Name='test_lighting_zero_exit'; Includes=@('-Itests/lighting_stubs', '-Iapplication', '-Imodules/lighting') },
        @{ Name='test_ws2812_recovery'; Includes=@('-Itests/ws2812_stubs', '-Ibsp/dispatch') },
        @{ Name='test_buzzer_status'; Includes=@('-Itests/buzzer_stubs', '-Iapplication', '-Ibsp/buzzer') },
        @{ Name='test_buzzer_patterns'; Includes=@('-Itests/buzzer_stubs', '-Ibsp/buzzer') },
        @{ Name='test_steer_disabled'; Includes=$common },
        @{ Name='test_steer_slew'; Includes=$common },
        @{ Name='test_oid_stop_guard'; Includes=$common },
        @{ Name='test_oid_reverse_guard'; Includes=$common },
        @{ Name='test_oid_stop_guard_field_cases'; Includes=$common },
        @{ Name='test_oid_diagnostics'; Includes=@('-Itests/oid_stubs', '-Ibsp/communication', '-Imodules/drive', '-Imodules/protocol') },
        @{ Name='test_oid_mode_recovery'; Includes=@('-Itests/oid_stubs', '-Ibsp/communication', '-Imodules/drive', '-Imodules/protocol') },
        @{ Name='test_rs485_frame_timeout'; Includes=@('-Itests/rs485_stubs', '-Ibsp/communication', '-Ibsp/dispatch') },
        @{ Name='test_chassis_control'; Includes=$fc },
        @{ Name='test_chassis_calibration'; Includes=$fc },
        @{ Name='test_pwm_input'; Includes=@('-Itests/pwm_stubs', '-Ibsp/dispatch') },
        @{ Name='test_encoder_timestamp'; Includes=@('-Itests/input_time_stubs', '-Ibsp/dispatch') }
    )
    foreach ($case in $cases) {
        $executable = Join-Path $testOutput ($case.Name + '.exe')
        $arguments = @('-std=c99', '-Wall', '-Wextra', '-Werror') + $case.Includes + @("tests/$($case.Name).c", '-o', $executable)
        & $compiler @arguments
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case.Name)" }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case.Name)" }
    }
    Write-Output "PASS: $($cases.Count)/$($cases.Count) host test programs. No firmware download or hardware access."
}
finally {
    Pop-Location
    $env:PATH = $previousPath
}
