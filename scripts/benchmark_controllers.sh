#!/usr/bin/env bash
set -eo pipefail

project_dir="${PROJECT_DIR:-/root/project}"
result_dir="${1:-${project_dir}/benchmark_results}"
disturbance_torque="${DISTURBANCE_TORQUE:-20.0}"
disturbance_duration="${DISTURBANCE_DURATION:-0.1}"
benchmark_duration="${BENCHMARK_DURATION:-8.0}"

source /opt/ros/kilted/setup.bash
source "${project_dir}/install/setup.bash"
set -u
mkdir -p "${result_dir}"

run_trial() {
    local controller_type="$1"
    local simulator_pid
    local controller_pid=""
    local benchmarker_pid=""

    setsid ros2 launch simulator headless.launch.py \
        >"${result_dir}/${controller_type}_simulator.log" 2>&1 &
    simulator_pid=$!

    cleanup_trial() {
        if [[ -n "${benchmarker_pid:-}" ]]; then
            kill -INT -- "-${benchmarker_pid}" 2>/dev/null || true
            wait "${benchmarker_pid}" 2>/dev/null || true
            benchmarker_pid=""
        fi
        if [[ -n "${controller_pid:-}" ]]; then
            kill -INT -- "-${controller_pid}" 2>/dev/null || true
            wait "${controller_pid}" 2>/dev/null || true
            controller_pid=""
        fi
        if [[ -n "${simulator_pid:-}" ]]; then
            kill -INT -- "-${simulator_pid}" 2>/dev/null || true
            for _ in $(seq 1 20); do
                if ! kill -0 "${simulator_pid}" 2>/dev/null; then
                    break
                fi
                sleep 0.1
            done
            kill -TERM -- "-${simulator_pid}" 2>/dev/null || true
            wait "${simulator_pid}" 2>/dev/null || true
            simulator_pid=""
        fi
    }
    trap cleanup_trial EXIT

    for _ in $(seq 1 200); do
        if [[ "$(ros2 topic type /clock 2>/dev/null || true)" == "rosgraph_msgs/msg/Clock" ]]; then
            break
        fi
        sleep 0.1
    done
    timeout 20 ros2 topic echo /clock --once >/dev/null

    for _ in $(seq 1 100); do
        if ros2 topic info /disturbance_torque 2>/dev/null \
            | grep -Eq 'Subscription count: [1-9]'; then
            break
        fi
        sleep 0.1
    done

    setsid ros2 run controller controller --ros-args \
        -p controller_type:="${controller_type}" \
        >"${result_dir}/${controller_type}_controller.log" 2>&1 &
    controller_pid=$!

    for _ in $(seq 1 100); do
        if ros2 node list 2>/dev/null | grep -qx /double_pendulum_controller; then
            break
        fi
        sleep 0.1
    done

    setsid ros2 run controller benchmarker --ros-args \
        -p log_name:="${controller_type}" \
        -p log_path:="${result_dir}/${controller_type}.csv" \
        -p disturbance_torque:="${disturbance_torque}" \
        -p disturbance_duration:="${disturbance_duration}" \
        -p duration:="${benchmark_duration}" \
        >"${result_dir}/${controller_type}_benchmarker.log" 2>&1 &
    benchmarker_pid=$!

    for _ in $(seq 1 100); do
        if ros2 node list 2>/dev/null | grep -qx /pendulum_benchmarker; then
            break
        fi
        sleep 0.1
    done

    # All nodes are matched before simulation time is allowed to advance.
    sleep 0.5
    gz service -s /world/pendulum_world/control \
        --reqtype gz.msgs.WorldControl \
        --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req 'pause: false' >/dev/null

    wait "${benchmarker_pid}"
    benchmarker_pid=""

    cleanup_trial
    trap - EXIT
}

run_trial pid
run_trial lqr
python3 "${project_dir}/scripts/analyze_controller_logs.py" \
    "${result_dir}/pid.csv" "${result_dir}/lqr.csv" \
    "${disturbance_duration}" | tee "${result_dir}/comparison.csv"
