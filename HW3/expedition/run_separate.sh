#!/usr/bin/env bash
# Независимый запуск модели: часы и каждый участник — отдельные процессы, каждый
# запускается своей командой (то же можно сделать вручную в разных терминалах).
# Использование: ./run_separate.sh [опции часов], например ./run_separate.sh -c tests/t1_ideal.cfg
# Сборка с безымянными семафорами: PROG=./expedition_u ./run_separate.sh
cd "$(dirname "$0")" || exit 1
prog=${PROG:-./expedition}
roles=$($prog "$@" -R list) || exit 1        # роли участников сценария

$prog "$@" -R clock &                        # часы создают мир и ждут участников
clock=$!
sleep 0.3
for r in $roles; do
    $prog -R "$r" > /dev/null &              # события участников видны у часов
done

trap 'kill -INT $clock 2>/dev/null' INT TERM # Ctrl+C — часам (и всей группе от терминала)
wait $clock; code=$?
while kill -0 $clock 2>/dev/null; do wait $clock; code=$?; done
wait                                         # дождаться завершения участников
exit $code
