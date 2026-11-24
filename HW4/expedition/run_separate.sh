#!/usr/bin/env bash
# Демонстрация с отдельными TCP-процессами. Аргументы: [число роботов [опции сервера]].
cd "$(dirname "$0")" || exit 1
robots=${1:-4}
if [ "$#" -gt 0 ]; then shift; fi
case $robots in
    [1-9]) ;;
    *) echo "Число роботов должно быть от 1 до 9" >&2; exit 1 ;;
esac
ip=${EXP_IP:-127.0.0.1}
port=${EXP_PORT:-5054}
tmp=$(mktemp -d) || exit 1
trap 'rm -f "$tmp/server.log"; rmdir "$tmp"' EXIT

./expedition server "$ip" "$port" "$@" > "$tmp/server.log" &
server=$!
trap 'kill -INT "$server" 2>/dev/null' INT TERM
while ! grep -q 'СЕРВЕР   готов' "$tmp/server.log"; do
    if ! kill -0 "$server" 2>/dev/null; then wait "$server"; exit $?; fi
    sleep 0.05
done
./expedition watch "$ip" "$port" -m &
watch=$!
while ! grep -q 'наблюдатель подключён' "$tmp/server.log"; do
    if ! kill -0 "$server" 2>/dev/null; then wait "$server"; exit $?; fi
    if ! kill -0 "$watch" 2>/dev/null; then kill -TERM "$server"; wait "$server"; exit 1; fi
    sleep 0.05
done

for n in $(seq 1 "$robots"); do
    ./expedition robot "$ip" "$port" "$n" > /dev/null &
done
wait "$server"; code=$?
if [ "$code" -gt 128 ]; then wait "$server"; code=$?; fi
wait
exit "$code"
