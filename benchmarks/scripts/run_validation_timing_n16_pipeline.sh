#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

RESULTS_DIR="benchmarks/results"
PIPELINE_LOG="${RESULTS_DIR}/validation_timing_panda_cage_n16_pipeline.log"
mkdir -p "${RESULTS_DIR}"

exec > >(tee -a "${PIPELINE_LOG}") 2>&1

timestamp() {
    date +"%Y-%m-%d %H:%M:%S"
}

log() {
    printf '\n[%s] %s\n' "$(timestamp)" "$*"
}

valid_json_file() {
    local path="$1"
    [[ -s "${path}" ]] || return 1
    python3 -c 'import json, sys; json.load(open(sys.argv[1]))' "${path}" >/dev/null
}

trace_has_records() {
    local path="$1"
    [[ -s "${path}" ]] || return 1
    python3 -c 'import json, sys; d=json.load(open(sys.argv[1])); assert len(d.get("records", [])) > 0' "${path}" >/dev/null
}

metrics_complete() {
    local path="$1"
    [[ -s "${path}" ]] || return 1
    python3 -c 'import json, sys; d=json.load(open(sys.argv[1])); assert d.get("trace", {}).get("motion_only") is True; assert d.get("variants")' "${path}" >/dev/null
}

run_logged() {
    local step_log="$1"
    shift
    log "RUN $*"
    "$@" 2>&1 | tee "${step_log}"
}

trace_for_seed() {
    local seed="$1"
    local env_name="$2"
    local trace_path="$3"
    local trace_log="$4"
    shift 4

    if trace_has_records "${trace_path}"; then
        log "SKIP existing ${env_name} trace seed ${seed}: ${trace_path}"
        return
    fi

    run_logged "${trace_log}" \
        ./build/apps/validation_timing \
            --mode trace \
            --num-robots 16 \
            --task-index 0 \
            --seeds "${seed}" \
            --iterations 1000 \
            --time-limit 1000 \
            --trace-output "${trace_path}" \
            "$@"

    trace_has_records "${trace_path}"
}

replay_motion_only() {
    local env_name="$1"
    local trace_input="$2"
    local metrics_path="$3"
    local replay_log="$4"
    shift 4

    if metrics_complete "${metrics_path}"; then
        log "SKIP existing ${env_name} motion-only metrics: ${metrics_path}"
        return
    fi

    run_logged "${replay_log}" \
        ./build/apps/validation_timing \
            --mode replay \
            --num-robots 16 \
            --task-index 0 \
            --trace-input "${trace_input}" \
            --motion-only \
            --progress-interval 50 \
            --metrics-json "${metrics_path}" \
            "$@"

    metrics_complete "${metrics_path}"
}

log "Starting n=16 validation timing pipeline"
log "Generating traces first: obstacles seeds 0,1,2, then empty seeds 0,1,2"

for seed in 0 1 2; do
    trace_for_seed \
        "${seed}" \
        "obstacle" \
        "${RESULTS_DIR}/validation_timing_panda_cage_n16_seed${seed}_exact1000_trace.json" \
        "${RESULTS_DIR}/validation_timing_panda_cage_n16_seed${seed}_exact1000_trace.log"
done

for seed in 0 1 2; do
    trace_for_seed \
        "${seed}" \
        "empty" \
        "${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_seed${seed}_exact1000_trace.json" \
        "${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_seed${seed}_exact1000_trace.log" \
        --empty-environment
done

OBSTACLE_TRACES="${RESULTS_DIR}/validation_timing_panda_cage_n16_seed0_exact1000_trace.json,${RESULTS_DIR}/validation_timing_panda_cage_n16_seed1_exact1000_trace.json,${RESULTS_DIR}/validation_timing_panda_cage_n16_seed2_exact1000_trace.json"
EMPTY_TRACES="${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_seed0_exact1000_trace.json,${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_seed1_exact1000_trace.json,${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_seed2_exact1000_trace.json"

log "Trace generation complete. Starting motion-only validation replays: obstacles first, empty second"

replay_motion_only \
    "obstacle" \
    "${OBSTACLE_TRACES}" \
    "${RESULTS_DIR}/validation_timing_panda_cage_n16_motion_only_exact1000_metrics.json" \
    "${RESULTS_DIR}/validation_timing_panda_cage_n16_motion_only_exact1000.log"

replay_motion_only \
    "empty" \
    "${EMPTY_TRACES}" \
    "${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_motion_only_exact1000_metrics.json" \
    "${RESULTS_DIR}/validation_timing_panda_cage_n16_empty_motion_only_exact1000.log" \
    --empty-environment

log "n=16 validation timing pipeline complete"
