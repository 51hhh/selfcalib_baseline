#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
projects_dir=$(dirname "$repo_dir")
rscalib_dir="$projects_dir/rscalib"
collection_root="/home/rick/desktop/video/sdcard"
dataset_arg=""
method="swift"
config=""
output_root=""
run_id="$(date -u +%Y%m%dT%H%M%SZ)"
rscalib_build="${RSCALIB_BUILD_DIR:-$rscalib_dir/build_standalone}"
kalibr_image="${KALIBR_HUAI_IMAGE:-kalibr:huai}"
read -r -a docker_cmd <<< "${DOCKER:-docker}"
dry_run=0

usage() {
  cat <<'EOF'
Usage: ./run_dataset.sh --dataset <id-or-directory> [options]

Options:
  --method swift|karpenko|ctrlvio  Baseline to run (default: swift)
  --config FILE          Override method configuration/camera YAML
                         (required for A350 Ctrl-VIO)
  --collection-root DIR  Dataset collection (default: /home/rick/desktop/video/sdcard)
  --output-root DIR      Output root (default: <collection>/outputs)
  --run-id ID            Immutable run name
  --rscalib-build DIR    Build containing native preparation tools
  --dry-run              Check required paths/executables and print the plan;
                         does not validate manifest contents or media integrity
EOF
}

while (($#)); do
  case "$1" in
    --dataset) dataset_arg=${2:?missing value}; shift 2 ;;
    --method) method=${2:?missing value}; shift 2 ;;
    --config) config=${2:?missing value}; shift 2 ;;
    --collection-root) collection_root=${2:?missing value}; shift 2 ;;
    --output-root) output_root=${2:?missing value}; shift 2 ;;
    --run-id) run_id=${2:?missing value}; shift 2 ;;
    --rscalib-build) rscalib_build=${2:?missing value}; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$method" in swift|karpenko|ctrlvio) ;; *) echo "Unknown method: $method" >&2; exit 2 ;; esac
[[ -n "$dataset_arg" ]] || { usage >&2; exit 2; }
if [[ -d "$dataset_arg" ]]; then dataset_dir=$(realpath "$dataset_arg");
else dataset_dir=$(realpath "$collection_root/datasets/$dataset_arg"); fi
dataset_id=$(basename "$dataset_dir")
output_root=${output_root:-$collection_root/outputs}
run_dir="$output_root/$dataset_id/selfcalib_baseline/$run_id"
work_dir="$run_dir/work"
result_dir="$run_dir/results/$method"
log_dir="$run_dir/logs"

raw_video="$dataset_dir/raw/video.mp4"
raw_camera="$dataset_dir/raw/data_Video.txt"
raw_imu="$dataset_dir/raw/data_Gyro.txt"
for input in "$dataset_dir/manifest.yaml" "$raw_video" "$raw_camera" "$raw_imu"; do
  [[ -f "$input" ]] || { echo "Missing canonical input: $input" >&2; exit 1; }
done
prepare="$rscalib_build/src/cli/rscalib_prepare"
prepare_imu="$rscalib_build/src/cli/rscalib_prepare_imu"
align="$rscalib_build/src/cli/rscalib_align"
for tool in "$prepare_imu" "$align"; do
  [[ -x "$tool" ]] || { echo "Missing executable: $tool" >&2; exit 1; }
done

if [[ "$method" == swift ]]; then
  config=${config:-$repo_dir/swift_vio/config/config_real_rs_mono.yaml}
elif [[ "$method" == karpenko ]]; then
  config=${config:-$rscalib_dir/config/camera_outdoor_reference_huai.yaml}
  [[ -x "$prepare" ]] || { echo "Missing executable: $prepare" >&2; exit 1; }
  [[ -x "$repo_dir/karpenko/build/karpenko_calib" ]] || { echo "Build karpenko first: $repo_dir/karpenko/build/karpenko_calib" >&2; exit 1; }
  command -v ffmpeg >/dev/null || { echo "ffmpeg not found" >&2; exit 1; }
else
  [[ -n "$config" ]] || {
    echo "Ctrl-VIO has no audited A350 configuration; pass --config explicitly." >&2
    exit 2
  }
fi
config=$(realpath "$config")
[[ -f "$config" ]] || { echo "Missing config: $config" >&2; exit 1; }

imu_cmd=$(printf '%q ' "$prepare_imu" --input "$raw_imu" --output "$work_dir/imu_si.txt" --manifest "$work_dir/imu_prepare.yaml")
if [[ "$method" == karpenko ]]; then
  video_for_alignment="$work_dir/video_3840x2880.mp4"
  prepare_cmd=$(printf '%q ' "$prepare" --video "$raw_video" --out-video "$video_for_alignment" --manifest "$work_dir/video_prepare.yaml")
else
  video_for_alignment="$raw_video"
  prepare_cmd="none (method uses the recorded 3840x2160 domain)"
fi
align_cmd=$(printf '%q ' "$align" --video "$video_for_alignment" --metadata "$raw_camera" --out-times "$work_dir/video_sof_seconds.txt" --out-frames "$work_dir/video_frames.csv" --manifest "$work_dir/video_alignment.yaml")

echo "dataset: $dataset_dir"
echo "method:  $method"
echo "config:  $config"
echo "output:  $run_dir"
echo "command: $prepare_cmd"
echo "command: $imu_cmd"
echo "command: $align_cmd"
if [[ "$method" == karpenko ]]; then
  echo "command: ffmpeg frame extraction -> $work_dir/frames (timestamps remain in the aligned table)"
  echo "command: karpenko_calib --enum-perm -> $result_dir/result.yaml"
else
  echo "container: build audited ROS bag adapter, pack aligned video/IMU, run $method"
fi
((dry_run)) && exit 0

[[ ! -e "$run_dir" ]] || { echo "Run already exists; refusing to overwrite: $run_dir" >&2; exit 1; }
mkdir -p "$work_dir" "$result_dir" "$log_dir"
{
  echo "schema: a350.project_run/v1"
  echo "project: selfcalib_baseline"
  echo "method: $method"
  echo "dataset_id: $dataset_id"
  echo "dataset_manifest: $dataset_dir/manifest.yaml"
  echo "dataset_manifest_sha256: $(sha256sum "$dataset_dir/manifest.yaml" | awk '{print $1}')"
  echo "config: $config"
  echo "config_sha256: $(sha256sum "$config" | awk '{print $1}')"
  echo "git_commit: $(git -C "$repo_dir" rev-parse HEAD)"
  echo "created_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "status: running"
  echo "raw_inputs_modified: false"
} > "$run_dir/run_manifest.yaml"

if [[ "$method" == karpenko ]]; then
  bash -o pipefail -c "$prepare_cmd" 2>&1 | tee "$log_dir/01_prepare.log"
fi
bash -o pipefail -c "$imu_cmd" 2>&1 | tee "$log_dir/02_prepare_imu.log"
bash -o pipefail -c "$align_cmd" 2>&1 | tee "$log_dir/03_align.log"

if [[ "$method" == karpenko ]]; then
  mkdir -p "$work_dir/frames"
  ffmpeg -hide_banner -loglevel warning -hwaccel auto -i "$video_for_alignment" \
    -fps_mode passthrough -start_number 0 "$work_dir/frames/%06d.png" \
    2>&1 | tee "$log_dir/04_extract_frames.log"
  "$repo_dir/karpenko/build/karpenko_calib" --cam "$config" --frames "$work_dir/frames" \
    --ts "$work_dir/video_sof_seconds.txt" --imu "$work_dir/imu_si.txt" \
    --out "$result_dir/result.yaml" --enum-perm 2>&1 | tee "$log_dir/05_karpenko.log"
else
  command -v "${docker_cmd[0]}" >/dev/null || { echo "Docker command not found: ${docker_cmd[0]}" >&2; exit 1; }
  container_base=(run --rm --user "$(id -u):$(id -g)" --entrypoint bash
    -e HOME=/tmp -v "$projects_dir:/projects:ro" -v "$run_dir:/run"
    -v "$video_for_alignment:/input/video.mp4:ro" "$kalibr_image" -lc)
  compile_script='source /opt/ros/noetic/setup.bash; g++ -std=c++17 -O2 /projects/rscalib/tools/ros1/rscalib_to_rosbag.cpp -o /run/work/rscalib_to_rosbag $(pkg-config --cflags --libs opencv4 rosbag sensor_msgs)'
  "${docker_cmd[@]}" "${container_base[@]}" "$compile_script" 2>&1 | tee "$log_dir/04_build_bag_adapter.log"
  video_sha=$(sha256sum "$video_for_alignment" | awk '{print $1}')
  times_sha=$(sha256sum "$work_dir/video_sof_seconds.txt" | awk '{print $1}')
  imu_sha=$(sha256sum "$work_dir/imu_si.txt" | awk '{print $1}')
  bag_script="source /opt/ros/noetic/setup.bash; /run/work/rscalib_to_rosbag --video /input/video.mp4 --times /run/work/video_sof_seconds.txt --imu /run/work/imu_si.txt --output /run/work/input.bag --manifest /run/work/input.bag.yaml --timestamp-source rdkx5_EISAlgoFrameInput.sofTimeStamp --timestamp-anchor unknown --input-beta 0 --exposure-source data_Video.txt.column2_not_applied --video-sha256 $video_sha --times-sha256 $times_sha --imu-sha256 $imu_sha"
  "${docker_cmd[@]}" "${container_base[@]}" "$bag_script" 2>&1 | tee "$log_dir/05_pack_rosbag.log"
  if [[ "$method" == swift ]]; then
    SELFCALIB_OUTPUT_DIR="$result_dir" DOCKER="${docker_cmd[*]}" \
      "$repo_dir/swift_vio/run_swift_vio.sh" "$work_dir/input.bag" "$config" /cam0/image_raw '' /imu0 \
      2>&1 | tee "$log_dir/06_swift.log"
  else
    SELFCALIB_OUTPUT_DIR="$result_dir" CTRLVIO_CONFIG="$config" DOCKER="${docker_cmd[*]}" \
      "$repo_dir/ctrlvio/run_ctrlvio.sh" "$work_dir/input.bag" 2>&1 | tee "$log_dir/06_ctrlvio.log"
  fi
fi

sed -i 's/^status: running$/status: complete/' "$run_dir/run_manifest.yaml"
echo "completed_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$run_dir/run_manifest.yaml"
echo "Completed: $run_dir"
