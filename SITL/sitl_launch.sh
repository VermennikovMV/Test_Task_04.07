DIR="$(cd "$(dirname "$0")" && pwd)"
AP="$HOME/ardupilot"
BIN="$AP/build/sitl/bin/arducopter"

if [ ! -x "$BIN" ]; then
    echo "=== Сборка SITL (первый раз - несколько минут) ==="
    source "$HOME/venv-ardupilot/bin/activate"
    cd "$AP"
    ./waf configure --board sitl && ./waf copter || exit 1
fi

# весь рабочий хлам SITL (eeprom.bin, logs/, terrain/) - в отдельную подпапку
mkdir -p "$DIR/sitl_run"
cd "$DIR/sitl_run"
echo "=== SITL стартует. Порты: 5760 (мост) / 5762 (Mission Planner). Ctrl+C - стоп ==="
# -w: чистый eeprom каждый запуск -> параметры всегда ровно из файла (воспроизводимость)
exec "$BIN" -w --model + --speedup 1 -I0 --defaults "$DIR/vio_params.parm"
