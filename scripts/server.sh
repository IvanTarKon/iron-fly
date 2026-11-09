#!/usr/bin/env bash
# Persistent local service, separate from the terminal/chat lifecycle.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
UNIT=muha-simulation.service
ACTION=${1:-start}
case "$ACTION" in
  start)
    if systemctl --user is-active --quiet "$UNIT"; then
      echo 'Сервер уже работает: http://127.0.0.1:8766'
      exit 0
    fi
    PYTHON_BIN=${PYTHON:-"$ROOT/.venv/bin/python"}
    if [ ! -x "$PYTHON_BIN" ]; then
      echo 'Укажите Python окружения: PYTHON=/absolute/path/to/python bash scripts/server.sh start' >&2
      exit 2
    fi
    systemd-run --user --unit="$UNIT" --collect \
      --description='Железная муха — локальная симуляция' \
      --working-directory="$ROOT" \
      --property=Restart=on-failure --property=RestartSec=2 \
      --property=TimeoutStopSec=10 \
      "$PYTHON_BIN" -u "$ROOT/scripts/simulation_ui.py" --port 8766
    echo 'Адрес: http://127.0.0.1:8766 (загрузка модели занимает несколько секунд)'
    ;;
  stop) systemctl --user stop "$UNIT" ;;
  restart) systemctl --user restart "$UNIT" ;;
  status) systemctl --user status "$UNIT" --no-pager ;;
  logs) journalctl --user -u "$UNIT" -n 60 --no-pager ;;
  *) echo 'Использование: server.sh start|stop|restart|status|logs' >&2; exit 2 ;;
esac
