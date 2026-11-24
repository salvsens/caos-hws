#!/usr/bin/env bash
# Простые проверки кодов, событий и сетевых ролей. Запуск: make test.
cd "$(dirname "$0")/.." || exit 1
make -s expedition || exit 1
ip=127.0.0.1
port=${EXP_TEST_PORT:-$((20000 + $$ % 20000))}
tmp=$(mktemp -d) || exit 1
pids=()
pass=0; fail=0

cleanup() {
    if [ "${#pids[@]}" -gt 0 ]; then kill -TERM "${pids[@]}" 2>/dev/null; fi
    wait
}
trap 'cleanup; rm -f "$tmp"/*.log; rmdir "$tmp"' EXIT
trap 'exit 1' INT TERM

result() {
    if [ "$2" -eq 1 ]; then pass=$((pass + 1)); echo "ПРОЙДЕН  $1 ($3)"
    else fail=$((fail + 1)); echo "ОШИБКА   $1 ($3)"; fi
}

wait_text() {
    local file=$1 pattern=$2 count=${3:-1}
    for ((i=0; i<100; i++)); do
        if [ -f "$file" ] && [ "$(grep -Fc -- "$pattern" "$file")" -ge "$count" ]; then return 0; fi
        sleep 0.03
    done
    return 1
}

start() {
    local file=$1 role=$2; shift 2
    ./expedition "$role" "$ip" "$port" "$@" > "$file" 2>&1 &
    pid=$!
    pids+=("$pid")
}

scenario() {
    local name=$1; shift
    port=$((port + 1))
    start "$tmp/server.log" server -d 0 -c "tests/$name.cfg" -l "tests/$name.log" "$@"
    server=$pid
}

robots() {
    local count=$1
    workers=()
    for ((n=1; n<=count; n++)); do
        start "$tmp/robot$n.log" robot "$n"
        workers+=("$pid")
    done
}

run_case() {
    local name=$1; shift
    local count
    count=$(awk '$1 == "robot" { n++ } END { print n+0 }' "tests/$name.cfg")
    scenario "$name" "$@"
    if [ "$name" != t8_bad ]; then
        if ! wait_text "$tmp/server.log" 'СЕРВЕР   готов'; then cleanup; pids=(); return 99; fi
        robots "$count"
    else
        workers=()
    fi
    local got code good=1
    wait "$server"; got=$?
    for worker in "${workers[@]}"; do
        wait "$worker"; code=$?
        [ "$code" -eq "$got" ] || good=0
    done
    pids=()
    [ "$good" -eq 1 ] && return "$got"
    return 99
}

check() {   # check ИМЯ КОД [строки журнала...]
    local name=$1 want=$2; shift 2
    rm -f "tests/$name.log"
    run_case "$name"
    local got=$? ok=1
    [ "$got" -eq "$want" ] || ok=0
    grep -Fq 'НАРУШЕН ИНВАРИАНТ' "tests/$name.log" 2>/dev/null && ok=0
    for line in "$@"; do
        grep -Fq -- "$line" "tests/$name.log" || { ok=0; echo "    нет «$line»"; }
    done
    result "$name" "$ok" "код $got, ожидался $want"
}

check t1_ideal       0 "ВЫПОЛНЕНО"
check t2_lossy       0 "ПОВТОР" "дубликат" "потеряно в канале"
check t3_unavailable 0 "нет робота с приборами MAG" "не хватает памяти"
check t4_energy      0 "встал на зарядку"
check t5_exhausted   3 "запас энергии исчерпан" "ресурсы экспедиции исчерпаны"
check t6_ticks       2 "предел тактов"
check t8_bad         1
check t9_queue       0 "ждёт в очереди" "разъём свободен"

seen=0
for seed in $(seq 1 10); do
    if run_case t7_failure -s "$seed" && grep -Fq 'АВАРИЯ' tests/t7_failure.log &&
       grep -Fq '(повторно)' tests/t7_failure.log; then seen=$((seen + 1)); fi
done
if [ "$seen" -gt 0 ]; then result t7_failure 1 "авария и повторное назначение в $seen из 10"
else result t7_failure 0 "ни одной аварии с повторным назначением"; fi

# SIGINT серверу: оба робота и наблюдатель получили итоговый код, отчёт сохранён.
sigint() {
    scenario t2_lossy -t 0 -d 80 -l tests/sigint.log
    wait_text "$tmp/server.log" 'СЕРВЕР   готов' || return 1
    start "$tmp/watch.log" watch; local observer=$pid
    wait_text "$tmp/server.log" 'наблюдатель подключён' || return 1
    robots 2
    wait_text "$tmp/server.log" 'подключились все' || return 1
    kill -INT "$server" || return 1
    wait "$server"; local code=$?
    [ "$code" -eq 4 ] || return 1
    for worker in "$observer" "${workers[@]}"; do
        wait "$worker"; code=$?
        [ "$code" -eq 4 ] || return 1
    done
    pids=()
    grep -Fq 'ИТОГИ' "$tmp/watch.log" && grep -Fq 'получен сигнал' tests/sigint.log
}

# Выход робота по сигналу, контроль доступа и повторный вход обоих типов клиентов.
reconnect() {
    scenario t1_ideal -d 80 -l tests/reconnect.log
    wait_text "$tmp/server.log" 'СЕРВЕР   готов' || return 1
    start "$tmp/watch1.log" watch -m; local first=$pid
    start "$tmp/watch2.log" watch; local second=$pid
    wait_text "$tmp/server.log" 'наблюдатель подключён' 2 || return 1
    robots 2
    wait_text "$tmp/watch1.log" 'такт 2 ' || return 1
    kill -INT "$first" "${workers[1]}" || return 1
    for worker in "$first" "${workers[1]}"; do
        wait "$worker"
        [ "$?" -eq 4 ] || return 1
    done
    wait_text "$tmp/server.log" 'R2 отключён' || return 1
    start "$tmp/robot2-back.log" robot 2; local r2=$pid
    wait_text "$tmp/server.log" 'R2 подключился снова' || return 1

    ./expedition control "$ip" "$port" off 1 > "$tmp/control.log" 2>&1 || return 1
    wait "${workers[0]}" || return 1
    grep -Fq 'отключён командой управления' "$tmp/robot1.log" || return 1
    ./expedition robot "$ip" "$port" 1 > "$tmp/rejected.log" 2>&1
    [ "$?" -eq 1 ] && grep -Fq 'вход этого робота запрещён' "$tmp/rejected.log" || return 1

    start "$tmp/watch3.log" watch -m; local third=$pid
    wait_text "$tmp/watch3.log" 'клиент отключён' || return 1
    ./expedition control "$ip" "$port" on 1 >> "$tmp/control.log" 2>&1 || return 1
    start "$tmp/robot1-back.log" robot 1; local r1=$pid
    wait_text "$tmp/server.log" 'R1 подключился снова' || return 1
    for worker in "$server" "$r1" "$r2" "$second" "$third"; do wait "$worker" || return 1; done
    pids=()
    grep -Fq 'ИТОГИ' "$tmp/watch2.log" && grep -Fq 'ИТОГИ' "$tmp/watch3.log" &&
        grep -Fq 'ВЫПОЛНЕНО' tests/reconnect.log && grep -Fq 'нарушений нет' tests/reconnect.log
}

if sigint; then result SIGINT 1 "код 4 у сервера, роботов и наблюдателя"
else result SIGINT 0 "см. журнал"; cleanup; pids=(); fi
if reconnect; then result "переподключение" 1 "два наблюдателя, SIGINT роботу, off/on, итоговый код 0"
else result "переподключение" 0 "см. журнал"; cleanup; pids=(); fi

echo "Итого: пройдено $pass, ошибок $fail"
[ "$fail" -eq 0 ]
