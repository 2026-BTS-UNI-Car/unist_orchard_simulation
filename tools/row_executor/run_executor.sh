#!/bin/sh
# UBTS row executor launcher (Jetson). NOT run by the development workflow; see README_FIELD.txt.
#
#   run_executor.sh check              image/ROS/numpy/scipy imports, SHA256SUMS, vendored hashes, config: no ROS node
#   run_executor.sh shadow [--auto]    SHADOW (default mode): the bridge socket directory is NOT mounted at all
#   run_executor.sh motor              MOTOR: mounts /run/ubts-rover (ESP bridge socket) READ-ONLY (connect works
#                                      through a ro bind); output still needs motor_output_enabled=true in
#                                      config.json AND an Arm on the operator page (calib_motor_output_enabled=true
#                                      instead: only the calibration runs arc/rolldown are armed, calib_runs.py)
#   run_executor.sh bench --wheels-off-ground-confirmed
#                                      WHEELS OFF THE GROUND only (V7): profile 'bench' from config/bench.json
#                                      (bench_ack WHEELS_OFF_GROUND, fixed duty pattern through the same hold/guard;
#                                      it does NOT look at obstacles or motion), MOTOR. Refused without the flag,
#                                      when bench.json is not profile "bench" (e.g. "field"), and when
#                                      config/config.json enables motor output (= the field setup). Delete
#                                      config/bench.json from the Jetson after V7 (README_FIELD.txt).
#   run_executor.sh stop | status | logs
#
# Layout on the Jetson (all paths can be overridden by environment variables):
#   $BASE=~/row-executor
#     src/row_executor/...        the package copied from tools/row_executor (incl. row_executor/SHA256SUMS)
#     SUMS_SHA256                 optional: sha256 of src/row_executor/SHA256SUMS recorded when it was copied
#     config/config.json          executor config (bind-mounted read-only)
#     IMAGE                       one line: the installed native observe image ID (sha256:...), never a tag
#     logs/                       JSONL logs + operator_token.txt (written by the container, root-owned)
# SHADOW containers are retained after exit so failed startup logs survive; only a verified owner may remove them.
# check/MOTOR/bench retain the existing --rm behaviour. All modes use --restart no,
# --network host (check: none) --ipc host --user 0:0 --security-opt no-new-privileges,
# ENTRYPOINT overridden (/bin/bash, ROS setup sourced explicitly), source and config read-only.
# Nothing here touches docker compose, the dashboard, the bridge, the drivers or the firmware.
set -eu

BASE=${UBTS_ROW_EXECUTOR_HOME:-$HOME/row-executor}
SRC=${UBTS_ROW_EXECUTOR_SRC:-$BASE/src}
CFG_DIR=${UBTS_ROW_EXECUTOR_CONFIG_DIR:-$BASE/config}
LOG_DIR=${UBTS_ROW_EXECUTOR_LOGS:-$BASE/logs}
IMAGE=${UBTS_ROW_EXECUTOR_IMAGE:-$(cat "$BASE/IMAGE" 2>/dev/null || true)}
NAME=ubts-row-executor
SOCKET_DIR=/run/ubts-rover

die() { echo "run_executor: $*" >&2; exit 2; }

cmd=${1:-}
[ -n "$cmd" ] || die "usage: run_executor.sh check|shadow [--auto]|motor|bench --wheels-off-ground-confirmed|stop|status|logs"
shift || true

if [ "$cmd" = bench ]; then
  # the bench pattern ignores obstacles and motion: an explicit confirmation on every start (review S4)
  [ "${1:-}" = "--wheels-off-ground-confirmed" ] || die "bench needs --wheels-off-ground-confirmed: ALL FOUR WHEELS OFF THE GROUND, E-stop in reach. The bench pattern does not look at obstacles or motion. Never in the orchard."
  shift
fi

case "$cmd" in
  stop)   exec docker stop -t 5 "$NAME" ;;
  status) exec docker ps -a --filter "name=^/${NAME}\$" --format '{{.Names}} {{.Status}} {{.Image}}' ;;
  logs)   exec docker logs --tail 50 -f "$NAME" ;;
  check|shadow|motor|bench) ;;
  *) die "unknown command $cmd" ;;
esac

# --- preconditions -------------------------------------------------------------------------------
case "$IMAGE" in
  sha256:*) ;;
  *) die "IMAGE must be an image ID sha256:... (file $BASE/IMAGE or UBTS_ROW_EXECUTOR_IMAGE), got '$IMAGE'" ;;
esac
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not present locally"
[ -f "$SRC/row_executor/SHA256SUMS" ] || die "missing $SRC/row_executor/SHA256SUMS"
( cd "$SRC/row_executor" && sha256sum -c --quiet --strict SHA256SUMS ) || die "source SHA mismatch in $SRC/row_executor"
if [ -f "$BASE/SUMS_SHA256" ]; then
  want=$(tr -d ' \n' < "$BASE/SUMS_SHA256")
  have=$(sha256sum "$SRC/row_executor/SHA256SUMS" | cut -d' ' -f1)
  [ "$want" = "$have" ] || die "SHA256SUMS differs from the recorded SUMS_SHA256 ($have != $want)"
fi
CFG_FILE=config.json
[ "$cmd" = bench ] && CFG_FILE=bench.json
[ -f "$CFG_DIR/$CFG_FILE" ] || die "missing $CFG_DIR/$CFG_FILE"
# "profile" of a JSON config (whitespace removed; the executor validates the file again in the container)
cfg_profile() { tr -d ' \t\r\n' < "$1" | sed -n 's/.*"profile":"\([A-Za-z_]*\)".*/\1/p'; }
if [ "$cmd" = bench ]; then
  bench_profile=$(cfg_profile "$CFG_DIR/bench.json")
  [ "$bench_profile" != field ] || die "bench.json has profile \"field\": the bench pattern is refused with a field config"
  [ "$bench_profile" = bench ] || die "bench.json must have profile \"bench\" (got '$bench_profile')"
  if [ -f "$CFG_DIR/config.json" ] && tr -d ' \t\r\n' < "$CFG_DIR/config.json" | grep -q '"motor_output_enabled":true'; then
    die "config/config.json enables motor output (field setup): bench refused. The bench is for V7 at home only."
  fi
  if [ -f "$CFG_DIR/config.json" ] && tr -d ' \t\r\n' < "$CFG_DIR/config.json" | grep -q '"calib_motor_output_enabled":true'; then
    die "config/config.json enables calibration motor output (field setup): bench refused. The bench is for V7 at home only."
  fi
fi
mkdir -p "$LOG_DIR"
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  die "a container named $NAME already exists (run_executor.sh status / stop)"
fi
echo "source SHA256SUMS: $(sha256sum "$SRC/row_executor/SHA256SUMS" | cut -d' ' -f1)"
echo "$CFG_FILE:       $(sha256sum "$CFG_DIR/$CFG_FILE" | cut -d' ' -f1)"
echo "image:             $IMAGE"

# ROS setup of the image + the read-only package on PYTHONPATH; python is PID 1 (exec) and handles SIGTERM.
PRELUDE='set -e; if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then . "/opt/ros/$ROS_DISTRO/setup.bash"; else for f in /opt/ros/*/setup.bash; do . "$f"; break; done; fi; export PYTHONPATH=/opt/row_executor${PYTHONPATH:+:$PYTHONPATH};'

COMMON="--name $NAME --restart no --ipc host --user 0:0 --security-opt no-new-privileges
 --entrypoint /bin/bash -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1
 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1
 -v $SRC:/opt/row_executor:ro -v $CFG_DIR:/etc/row_executor:ro -v $LOG_DIR:/var/log/row_executor"

case "$cmd" in
  check)
    # shellcheck disable=SC2086
    exec docker run --rm $COMMON --network none "$IMAGE" -c "$PRELUDE"'
      python3 -B -s - <<"EOF"
import numpy, scipy, rclpy, sensor_msgs.msg, std_msgs.msg
from scipy.spatial import cKDTree
from scipy.sparse.csgraph import connected_components
cKDTree(numpy.zeros((4, 2))).query_pairs(1., output_type="ndarray")
from row_executor import executor_main as M, core, config as C
digest = M.verify_sources()
manifest = core.verify_vendor()
core.Runner(C.ExecutorConfig())
cfg = C.load_config("/etc/row_executor/config.json")
print("OK numpy", numpy.__version__, "scipy", scipy.__version__, "sums", digest[:12],
      "vendor", manifest["source_commit"][:12], "config", cfg.digest()[:12], "motor_output_enabled",
      cfg.motor_output_enabled, "calib_motor_output_enabled", cfg.calib_motor_output_enabled,
      "feasibility", cfg.feasibility()["status"])
EOF'
    ;;
  shadow)
    extra=""
    [ "${1:-}" = "--auto" ] && extra="--shadow-auto"
    # SHADOW: no -v $SOCKET_DIR (the bridge socket is unreachable from the container by construction)
    # shellcheck disable=SC2086
    shadow_cid=$(docker run -d $COMMON --network host "$IMAGE" -c "$PRELUDE"' exec python3 -B -s -m row_executor.executor_main --config /etc/row_executor/config.json --log-dir /var/log/row_executor "$@"' row-executor $extra) || die 'SHADOW docker start failed'
    printf '%s\n' "$shadow_cid" | grep -Eq '^[0-9a-f]{64}$' || die 'SHADOW start did not return one exact container ID; preserve evidence'
    printf '%s\n' "$shadow_cid"
    startup_log="$LOG_DIR/start-$shadow_cid.log"
    (umask 077; set -C; : > "$startup_log") || die 'refusing to overwrite existing startup evidence'
    sleep 2
    # Do not pipe docker logs through tail: POSIX sh would return tail's success
    # even if the container exited/vanished and Docker could not read any log.
    if ! docker logs --tail 80 "$shadow_cid" > "$startup_log" 2>&1; then
      die "SHADOW logs unavailable; container retained: $shadow_cid; evidence: $startup_log"
    fi
    running=$(docker inspect --format '{{.State.Running}}' "$shadow_cid") || die "SHADOW state unavailable; evidence: $startup_log"
    if [ "$running" != true ]; then
      tail -20 "$startup_log" >&2
      die "SHADOW exited before startup confirmation; retained: $shadow_cid; evidence: $startup_log"
    fi
    tail -20 "$startup_log"
    running=$(docker inspect --format '{{.State.Running}}' "$shadow_cid") || die "SHADOW state unavailable after logs; evidence: $startup_log"
    [ "$running" = true ] || die "SHADOW exited while reading startup logs; retained: $shadow_cid; evidence: $startup_log"
    echo "started $NAME (shadow). Startup evidence: $startup_log"
    echo 'SHADOW only: no vehicle control. Exited containers are retained; verify the exact owner/ID before removal.'
    echo "stop: $0 stop    follow: $0 logs"
    exit 0
    ;;
  motor)
    [ -S "$SOCKET_DIR/control.sock" ] || die "bridge socket $SOCKET_DIR/control.sock not found (is the ESP bridge running?)"
    # shellcheck disable=SC2086
    docker run -d --rm $COMMON --network host -v "$SOCKET_DIR:$SOCKET_DIR:ro" "$IMAGE" -c "$PRELUDE"' exec python3 -B -s -m row_executor.executor_main --config /etc/row_executor/config.json --log-dir /var/log/row_executor --motor-output "$@"' row-executor
    ;;
  bench)
    [ -S "$SOCKET_DIR/control.sock" ] || die "bridge socket $SOCKET_DIR/control.sock not found (is the ESP bridge running?)"
    echo "BENCH (V7): all four wheels OFF THE GROUND, physical E-stop in reach, nobody at the wheels."
    echo "BENCH: the duty pattern does not look at obstacles or motion. Never in the orchard."
    echo "BENCH: after V7 delete $CFG_DIR/bench.json from the Jetson (README_FIELD.txt 5)."
    # shellcheck disable=SC2086
    docker run -d --rm $COMMON --network host -v "$SOCKET_DIR:$SOCKET_DIR:ro" "$IMAGE" -c "$PRELUDE"' exec python3 -B -s -m row_executor.executor_main --config /etc/row_executor/bench.json --profile bench --log-dir /var/log/row_executor --motor-output "$@"' row-executor
    ;;
esac
echo "started $NAME ($cmd). Operator page URL (token) follows in the log; Ctrl-C stops following, not the executor."
sleep 2
docker logs "$NAME" 2>&1 | tail -20
echo "stop: $0 stop    follow: $0 logs"
