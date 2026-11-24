// =============================================================================
// view.c — журнал событий, карта наблюдателя и итоговый отчёт.
// =============================================================================
#include <stdarg.h>
#include <stdio.h>
#include "protocol.h"

void ev(World* w, int level, const char* fmt, ...) {
    char text[EVLEN - 16];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    unsigned k = w->ev_head++ % EVQ;
    w->evq[k].level = level;
    snprintf(w->evq[k].text, EVLEN, "[%04d] %s", w->tick % 10000, text);
}


static void append(Text* out, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

static void append(Text* out, const char* fmt, ...) {
    size_t space = sizeof out->data - out->len;
    if (space <= 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out->data + out->len, space, fmt, ap);
    va_end(ap);
    if (n > 0) out->len += (size_t)n < space ? (size_t)n : space - 1;
}

// Итоговый отчёт: причина, задания, недоставленные данные, статистика
void report(const World* w, Text* out, int code) {
    out->len = 0;
    static const char* why[] = {"", "выполнены все доступные задания", "достигнут предел тактов",
                                "ресурсы экспедиции исчерпаны", "прервано пользователем", "НАРУШЕН ИНВАРИАНТ"};
    static const char* st[] = {"ожидает", "назначено", "выполняется", "ВЫПОЛНЕНО", "недоступно"};
    append(out, "\n===== ИТОГИ ЭКСПЕДИЦИИ =====\nПричина завершения: %s %s\nТактов: %d, seed: %llu, роботов: %d\n",
            why[w->end], w->end == END_INVARIANT ? w->violation : "", w->tick, (unsigned long long)w->cfg.seed, w->nrob);
    append(out, "Задания:\n");
    for (int t = 0; t < w->ntask; t++) {
        const Task* k = &w->task[t];
        char ins[64];
        format_instr(k->instr, ins, sizeof ins);
        append(out, "  '%c' %-10s %-11s данные %d/%d, назначений %d", k->id, ins, st[k->st], k->nchunks, k->volume, k->attempts);
        if (k->st == T_DONE) append(out, ", такт %d\n", k->done_tick);
        else append(out, "%s%s\n", k->note[0] ? " — " : "", k->note);
    }
    append(out, "Невыполненные задания:");
    int un = 0;
    for (int t = 0; t < w->ntask; t++)
        if (w->task[t].st != T_DONE) { append(out, " '%c'", w->task[t].id); un++; }
    append(out, "%s\nНедоставленные данные:", un ? "" : " нет");
    int nd = 0;
    for (int i = 0; i < w->nrob; i++)
        for (int k = 0; k < w->rob[i].nmem; k++)
            if (!w->got[w->rob[i].mem[k].uid]) {
                append(out, " #%d('%c', у R%d)", w->rob[i].mem[k].uid, w->task[w->rob[i].mem[k].task].id, i + 1);
                nd++;
            }
    append(out, "%s\nРоботы:\n", nd ? "" : " нет");
    for (int i = 0; i < w->nrob; i++) {
        const Robot* r = &w->rob[i];
        append(out, "  R%d %-8s энергия %3d/%-3d шагов %3d, передач %2d (повторов %2d), получено энергии %3d, %s\n",
                i + 1, r->name, r->energy, r->emax, r->moves, r->sent, r->retries, r->charged, robot_state(r->st));
    }
    append(out, "Узлы связи (переслано/потеряно/продублировано):");
    for (int k = 0; k < w->nnode; k++)
        append(out, " %s%.0d %d/%d/%d;", k ? "^" : "B", k, w->node[k].fwd, w->node[k].lost, w->node[k].dup);
    append(out, "\n");
    append(out, "Связь: отправлено назначений/пакетов/подтверждений %d/%d/%d, потеряно %d/%d/%d, продублировано %d/%d/%d\n",
            w->sent[0], w->sent[1], w->sent[2], w->lost[0], w->lost[1], w->lost[2], w->dupd[0], w->dupd[1], w->dupd[2]);
    append(out, "База: засчитано уникальных пакетов %d, отброшено дубликатов %d\n", w->unique, w->dups);
    append(out, "Инварианты: проверок %d, %s\nКод завершения: %d\n", w->checks,
            w->end == END_INVARIANT ? "есть нарушение" : "нарушений нет", code);
}

void draw(const World* w, Text* out, bool verbose) {
    out->len = 0;
    append(out, "== Планетарная экспедиция == такт %d (лимит %d), seed %llu\n",
           w->tick, w->cfg.max_ticks, (unsigned long long)w->cfg.seed);
    for (int r = 0; r < w->rows; r++) {
        char line[MAXC + 1];
        for (int c = 0; c < w->cols; c++) {
            char g = w->grid[r][c];
            for (int t = 0; t < w->ntask; t++)          // выполненная цель — заглавной буквой
                if (w->task[t].st == T_DONE && w->task[t].pos.r == r && w->task[t].pos.c == c) g = w->task[t].id - 32;
            for (int i = w->nrob - 1; i >= 0; i--)
                if (w->rob[i].pos.r == r && w->rob[i].pos.c == c) g = w->rob[i].st == R_FAILED ? 'X' : (char)('1' + i);
            line[c] = g;
        }
        line[w->cols] = '\0';
        append(out, "  %s\n", line);
    }
    append(out, "  # скалы ~ опасно B база C станция ^ ретранслятор a-z цели (A-Z выполнены) 1-9 роботы X авария\n");
    for (int i = 0; i < w->nrob; i++) {
        const Robot* r = &w->rob[i];
        append(out, "  R%d %-8s энергия %3d/%-3d память %d/%d задание %c  %s, %s, клиент %s\n", i + 1, r->name, r->energy,
                r->emax, r->nmem, r->mem_cap, r->task >= 0 ? w->task[r->task].id : '-',
                r->in_comm ? "связь" : "нет связи", robot_state(r->st), w->online[i] ? "в сети" : "отключён");
    }
    int done = 0, transit = 0;
    for (int t = 0; t < w->ntask; t++) done += w->task[t].st == T_DONE;
    for (int k = 0; k < w->nnode; k++) transit += w->node[k].nq;
    append(out, "  Задания: выполнено %d из %d | база: пакетов %d, дубликатов %d | в пути %d\n",
            done, w->ntask, w->unique, w->dups, transit);
    for (int c = 0; c < w->nch; c++) {
        int busy = 0;
        for (int i = 0; i < w->nrob; i++) busy += w->rob[i].st == R_CHARGE && w->rob[i].goal == c;
        append(out, "  C%d разъёмы %d/%d, запас ", c + 1, busy, w->cfg.charger_slots);
        if (w->chstock[c] < 0) append(out, "не ограничен\n");
        else append(out, "%d\n", w->chstock[c]);
    }
    unsigned recent[10], count = 0;
    for (unsigned i = 0; i < w->ev_head && i < EVQ && count < 10; i++) {
        unsigned k = (w->ev_head - i - 1) % EVQ;
        if (w->evq[k].level <= verbose) recent[count++] = k;
    }
    while (count) append(out, "  %s\n", w->evq[recent[--count]].text);
}
