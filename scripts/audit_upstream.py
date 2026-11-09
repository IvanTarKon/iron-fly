"""Classify the immutable baseline; run a heartbeat-only diagnostic variant."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / 'vendor/Hexapod-Simulation/Hexapod-Firmware'
source = (FW / 'test/test_robot.cpp').read_text()
old = 'static void pump(robot::Robot& r, int n) { for (int i = 0; i < n; ++i) r.update(DT); }'
new = 'static void pump(robot::Robot& r, int n) { for (int i = 0; i < n; ++i) { r.heartbeat(); r.update(DT); } }'
assert old in source
with tempfile.TemporaryDirectory() as tmp:
    probe = Path(tmp) / 'test_robot_heartbeat.cpp'
    probe.write_text(source.replace(old, new))
    exe = Path(tmp) / 'probe'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(FW / 'src'), '-I' + str(FW / 'test'), str(probe), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], text=True, capture_output=True)
    (ROOT / 'reports/heartbeat-audit.txt').write_text(result.stdout + result.stderr)
failures = re.findall(r'FAIL  (.*?)\s+\[test/test_robot.cpp:(\d+)\]',
                      (ROOT / 'reports/baseline/pinned-upstream-tests.txt').read_text())
assert len(failures) == 25
rows = []
for name, line in failures:
    n = int(line)
    if n == 86:
        reason = 'Конструктор записывает сложенную позу при отключённом питании.'
        action = 'Несовпадение ожидания; запись цели не означает включение приводов.'
    elif n == 132:
        reason = 'Штатный watchdog запускает посадку и выключение; тест ожидает стойку.'
        action = 'Сохранить upstream-политику; наш timeout проверять отдельно через адаптер.'
    elif n == 170:
        reason = 'Ядро намеренно не считает низкое напряжение самостоятельной аварией.'
        action = 'Не заявлять такую защиту; аппаратная политика вне виртуального MVP.'
    else:
        reason = 'pump(500) выполняет 10 с без heartbeat: робот успевает сесть/выключиться; последующие проверки каскадно неверны.'
        action = 'Диагностический вариант с heartbeat проходит эту проверку; оригинал не менять.'
    rows.append(f'| {line} | {name} | {reason} | {action} |')
text = '''# Аудит 25 исходных сбоев test_robot

Исходный лог сохранён в `baseline/pinned-upstream-tests.txt`; upstream-тесты не изменены.
Диагностический вариант меняет только `pump`: heartbeat перед каждым update.
Это исследование причин, а не замена исходной приёмки.

| Строка test_robot.cpp | Проверка | Причина | Решение |
|---|---|---|---|
''' + '\n'.join(rows) + '''

После изменения только pump остаются четыре ошибки: запись цели в конструкторе,
состояние после watchdog, низкое напряжение и отключение питания при нём.
Последняя проверка раньше случайно проходила: питание уже отключилось по watchdog.
Следовательно, 22 из 25 исходных сбоев исчезает после исправления тестового поддержания
связи; три исходных относятся к изменённому поведению. Это не 25 независимых дефектов.

Наши проверки отдельно подтверждают stop в стойке, блокировку после ошибочной/просроченной
команды, явный start, sticky over-current FAULT, все три походки и отображение 18 каналов.
Минимальный патч resolve не меняет watchdog, напряжение или ожидания upstream-тестов.
'''
(ROOT / 'reports/test_robot_audit.md').write_text(text)
expected = ['no servo writes before enable', 'watchdog stopped the robot',
            'low-voltage -> FAULT', 'power cut on FAULT']
actual = re.findall(r'FAIL  (.*?)\s+\[', result.stdout)
if actual != expected or result.returncode != 1:
    raise SystemExit('Unexpected heartbeat audit result: ' + repr(actual))
print('25 baseline failures classified; heartbeat-only diagnostic: 4 expected failures')
