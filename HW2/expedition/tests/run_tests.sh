#!/usr/bin/env bash
# Автоматическая проверка модели. Запуск: make test
# Порядок действий потоков внутри такта недетерминирован, поэтому проверяются коды
# завершения и характерные события в журнале; авария (случайная) — серией seed.
cd "$(dirname "$0")/.." || exit 1
make -s expedition || exit 1
pass=0; fail=0

check() {   # check ИМЯ ОЖИДАЕМЫЙ_КОД [строка, которая должна быть в журнале]...
    local name=$1 want=$2; shift 2
    ./expedition -d 0 -c "tests/$name.cfg" -l "tests/$name.log" > /dev/null 2>&1
    local got=$?
    local ok=1
    [ "$got" -eq "$want" ] || ok=0
    grep -q "НАРУШЕН ИНВАРИАНТ" "tests/$name.log" 2>/dev/null && ok=0
    for s in "$@"; do grep -q -- "$s" "tests/$name.log" || { ok=0; echo "    нет «$s»"; }; done
    if [ $ok -eq 1 ]; then pass=$((pass + 1)); echo "ПРОЙДЕН  $name (код $got)"
    else fail=$((fail + 1)); echo "ОШИБКА   $name (код $got, ожидался $want)"; fi
}

check t1_ideal       0 "ВЫПОЛНЕНО"
check t2_lossy       0 "ПОВТОР" "дубликат" "потеряно в канале"
check t3_unavailable 0 "нет робота с приборами MAG" "не хватает памяти"
check t4_energy      0 "встал на зарядку"
check t5_exhausted   3 "запас энергии исчерпан" "ресурсы экспедиции исчерпаны"
check t6_ticks       2 "предел тактов"
check t8_bad         1
check t9_queue       0 "ждёт в очереди" "разъём свободен"

# Авария случайна: 10 seed, хотя бы в одном — авария и повторное назначение (код 0)
seen=0
for s in $(seq 1 10); do
    ./expedition -d 0 -s "$s" -c tests/t7_failure.cfg -l tests/t7_failure.log > /dev/null 2>&1 &&
        grep -q "АВАРИЯ" tests/t7_failure.log && grep -q "(повторно)" tests/t7_failure.log && seen=$((seen + 1))
done
if [ $seen -gt 0 ]; then pass=$((pass + 1)); echo "ПРОЙДЕН  t7_failure (авария и повторное назначение в $seen из 10)"
else fail=$((fail + 1)); echo "ОШИБКА   t7_failure: ни одной аварии с повторным назначением"; fi

# Режим без ограничения тактов: завершение по Ctrl+C (SIGINT) с сохранением итогов
./expedition -t 0 -d 50 -c tests/t2_lossy.cfg -l tests/sigint.log > /dev/null &
pid=$!; sleep 1; kill -INT $pid; wait $pid; code=$?
if [ $code -eq 4 ] && grep -q "ИТОГИ" tests/sigint.log; then pass=$((pass + 1)); echo "ПРОЙДЕН  SIGINT (код 4, итоги в журнале)"
else fail=$((fail + 1)); echo "ОШИБКА   SIGINT: код $code"; fi

echo "Итого: пройдено $pass, ошибок $fail"
[ $fail -eq 0 ]
