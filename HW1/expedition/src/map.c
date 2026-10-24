// =============================================================================
// map.c — территория: загрузка карты, стоимость клеток, поля расстояний.
// Символы: . равнина, # скалы, ~ опасная зона, B база, C зарядная станция,
// ^ ретранслятор, a..z цели. База и станции — «стоянки» (безопасные точки),
// где с разрешения владельца клетки могут стоять несколько роботов.
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "model.h"

static const Pos DIRS[4] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};   // С, В, Ю, З

int map_init(World* w, char* err, size_t n) {
    Config* c = &w->cfg;
    w->rows = c->rows;
    w->cols = 0;
    for (int r = 0; r < c->rows; r++)
        if ((int)strlen(c->map[r]) > w->cols) w->cols = (int)strlen(c->map[r]);
    int bases = 0;
    for (int r = 0; r < w->rows; r++)
        for (int k = 0; k < w->cols; k++) {
            char ch = k < (int)strlen(c->map[r]) ? c->map[r][k] : '#';   // короткие строки — скалы
            if (ch == ' ') ch = '.';
            if (!strchr(".#~BC^", ch) && !(ch >= 'a' && ch <= 'z')) {
                snprintf(err, n, "недопустимый символ '%c' в карте (строка %d, столбец %d)", ch, r, k);
                return -1;
            }
            w->grid[r][k] = ch;
            if (ch == 'B') { w->base = (Pos){r, k}; bases++; }
            if (ch == 'C' && w->nch < MAXCH) {
                w->ch[w->nch] = (Pos){r, k};
                w->chstock[w->nch++] = c->charger_stock;
            }
        }
    if (bases != 1) { snprintf(err, n, "на карте должна быть ровно одна база B"); return -1; }
    if (w->nch == 0) { snprintf(err, n, "на карте нет ни одной зарядной станции C"); return -1; }
    return 0;
}

bool passable(const World* w, Pos p) {
    return p.r >= 0 && p.r < w->rows && p.c >= 0 && p.c < w->cols && w->grid[p.r][p.c] != '#';
}

bool is_dock(const World* w, Pos p) { return w->grid[p.r][p.c] == 'B' || w->grid[p.r][p.c] == 'C'; }

// Расход энергии на вход в клетку
int cell_cost(const World* w, Pos p) {
    return w->cfg.move_cost + (w->grid[p.r][p.c] == '~' ? w->cfg.danger_cost : 0);
}

// Поле расстояний: d[p] — минимальный расход энергии на путь из p до ближайшей
// клетки из goals. Волновой алгоритм с повторной обработкой улучшенных клеток
// (стоимости положительны, поэтому процесс сходится к кратчайшим путям).
void field(const World* w, const Pos* goals, int ngoals, int d[MAXR][MAXC]) {
    static Pos queue[MAXR * MAXC];
    static bool inq[MAXR][MAXC];
    int head = 0, count = 0;
    for (int r = 0; r < MAXR; r++)
        for (int k = 0; k < MAXC; k++) { d[r][k] = INF; inq[r][k] = false; }
    for (int i = 0; i < ngoals; i++) {
        Pos g = goals[i];
        d[g.r][g.c] = 0;
        queue[(head + count++) % (MAXR * MAXC)] = g;
        inq[g.r][g.c] = true;
    }
    while (count > 0) {
        Pos v = queue[head];
        head = (head + 1) % (MAXR * MAXC);
        count--;
        inq[v.r][v.c] = false;
        int via = d[v.r][v.c] + cell_cost(w, v);   // шаг из соседа u в v стоит cell_cost(v)
        for (int k = 0; k < 4; k++) {
            Pos u = {v.r + DIRS[k].r, v.c + DIRS[k].c};
            if (!passable(w, u) || via >= d[u.r][u.c]) continue;
            d[u.r][u.c] = via;
            if (!inq[u.r][u.c]) {
                queue[(head + count++) % (MAXR * MAXC)] = u;
                inq[u.r][u.c] = true;
            }
        }
    }
}

// Следующая клетка кратчайшего пути по полю d (from — если пути нет или уже у цели)
Pos step_to(const World* w, int d[MAXR][MAXC], Pos from) {
    for (int k = 0; k < 4; k++) {
        Pos n = {from.r + DIRS[k].r, from.c + DIRS[k].c};
        if (passable(w, n) && d[n.r][n.c] < INF && d[n.r][n.c] + cell_cost(w, n) == d[from.r][from.c])
            return n;
    }
    return from;
}

// Конечная клетка кратчайшего пути из from
Pos field_end(const World* w, int d[MAXR][MAXC], Pos from) {
    for (int i = 0; i < MAXR * MAXC && d[from.r][from.c] > 0; i++) {
        Pos n = step_to(w, d, from);
        if (n.r == from.r && n.c == from.c) break;
        from = n;
    }
    return from;
}
